"""Browser-native MJPEG plus optional two-channel RTMP egress."""
import asyncio
import os
import shutil
import time
from pathlib import Path
from PIL import ImageGrab
import io


class Media:
    def __init__(self,config,processes,bridge,root):
        self.config=config;self.processes=processes;self.bridge=bridge;self.root=Path(root)
        self.rviz_jpeg=None;self.rviz_at=0;self.rviz_error=None;self.running=True;self.rtmp={};self.retry_at={}

    def geometry(self):
        """Xvfb 屏幕尺寸即推流尺寸；RViz 窗口左上角被推到屏幕外，可见区域恰好只剩 3D 渲染区。"""
        w,h=self.config['rviz_size'];chrome=self.config.get('rviz_chrome') or {}
        return w,h,int(chrome.get('left',25)),int(chrome.get('top',71)),int(chrome.get('right',25)),int(chrome.get('bottom',39))

    async def initialize(self):
        display=self.config['rviz_display'];w,h,cl,ct,cr,cb=self.geometry()
        self.processes.spawn('xvfb',['Xvfb',display,'-screen','0',f'{w}x{h}x24','-nolisten','tcp','-ac','-nocursor'])
        await asyncio.sleep(1)
        # RViz 退出时会回写配置文件，因此每次启动都用一份干净的运行副本。
        config_path=self.root/'runtime'/'rviz'/'console.rviz'
        config_path.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(self.root/'deploy'/'console.rviz',config_path)
        self.processes.spawn('rviz',['rviz2','-d',str(config_path)],{'DISPLAY':display,'QT_X11_NO_MITSHM':'1','LIBGL_ALWAYS_SOFTWARE':'1'})
        # RViz occupies a dedicated virtual display; kiosk capture can never recurse.
        await asyncio.sleep(2)
        self.processes.spawn('rviz-layout',['bash',str(self.root/'deploy'/'rviz-layout.sh'),str(w),str(h),str(cl),str(ct),str(cr),str(cb)],{'DISPLAY':display})

    async def capture_loop(self):
        w,h=self.config['rviz_size']
        while self.running:
            try:
                process=self.processes.jobs.get('rviz')
                if not process or process.poll() is not None:raise RuntimeError('RViz process exited')
                img=await asyncio.to_thread(ImageGrab.grab,xdisplay=self.config['rviz_display'])
                if img.size!=(w,h):img=img.crop((0,0,min(w,img.width),min(h,img.height)))
                def encode():
                    b=io.BytesIO();img.save(b,format='JPEG',quality=75);return b.getvalue()
                self.rviz_jpeg=await asyncio.to_thread(encode);self.rviz_at=time.time();self.rviz_error=None
            except Exception as e:self.rviz_error=str(e)
            await asyncio.sleep(1/self.config['video_fps'])

    def frame(self,channel):
        if channel=='rviz':return self.rviz_jpeg,self.rviz_at
        return self.bridge.camera_jpeg,self.bridge.camera_at

    async def reconfigure_rtmp(self):
        for channel in ('rviz','camera'):
            await self.processes.stop('rtmp-'+channel)
            url=self.config[channel+'_rtmp_url']
            if not url:self.rtmp[channel]='disabled';continue
            self.start_rtmp(channel)

    def start_rtmp(self,channel):
        url=self.config[channel+'_rtmp_url']
        self.retry_at[channel]=time.monotonic()+10
        fps=self.config['video_fps']
        args=['ffmpeg','-hide_banner','-loglevel','warning','-rw_timeout','5000000','-fflags','nobuffer','-f','mpjpeg','-i',f'http://127.0.0.1:{self.config["port"]}/api/streams/{channel}.mjpeg',
                  '-an','-vf','scale=trunc(iw/2)*2:trunc(ih/2)*2','-r',str(fps),'-c:v','libx264','-threads','2','-preset','ultrafast','-tune','zerolatency','-pix_fmt','yuv420p','-g',str(fps*2),'-b:v','1200k','-f','flv',url]
        self.processes.spawn('rtmp-'+channel,args);self.rtmp[channel]='starting'

    async def monitor(self):
        while self.running:
            for ch in ('rviz','camera'):
                p=self.processes.jobs.get('rtmp-'+ch)
                if self.config[ch+'_rtmp_url']:
                    self.rtmp[ch]='running' if p and p.poll() is None else 'failed'
                    if self.rtmp[ch]=='failed' and time.monotonic()>self.retry_at.get(ch,0):
                        _,stamp=self.frame(ch)
                        if time.time()-stamp<3:self.start_rtmp(ch)
            await asyncio.sleep(2)
