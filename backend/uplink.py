"""Compatible with the existing platform's device gateway HTTP envelopes."""
import asyncio
import time
import uuid
from datetime import datetime,timezone
import aiohttp


def iso():return datetime.now(timezone.utc).isoformat()


class Uplink:
    def __init__(self,config,snapshot):
        self.config=config;self.snapshot=snapshot;self.state='unconfigured';self.error=None;self.last_success=None
        self.boot_id=uuid.uuid4().hex;self.seq=0;self.pending=[];self.running=True

    async def run(self):
        timeout=aiohttp.ClientTimeout(total=6)
        async with aiohttp.ClientSession(timeout=timeout) as session:
            registered=None;backoff=1
            while self.running:
                url=self.config['platform_url'].rstrip('/');token=self.config['platform_token'];device=self.config['device_id']
                if not url or not token:
                    self.state='unconfigured';await asyncio.sleep(1);continue
                headers={'X-Device-Id':device,'X-Device-Token':token}
                try:
                    identity=(url,token,device)
                    if registered!=identity:
                        body={'schemaVersion':'1.0','deviceId':device,'bootId':self.boot_id,'appVersion':'1.0.0','adapterVersion':'ros2-humble-1','host':{'kind':'robot','os':'linux'},'capabilities':{'mapping':True,'navigation':True,'telemetry':True,'video':['mjpeg','rtmp']}}
                        async with session.post(url+'/api/devices/register',json=body,headers=headers) as r:
                            if r.status>=400:raise RuntimeError(f'register HTTP {r.status}')
                            await r.read()
                        registered=identity
                    self.seq+=1;payload=self.snapshot()
                    # Large geometry travels via map/stream APIs, not a one-second telemetry envelope.
                    for key in ('scan_points','path','imu_history'):payload.pop(key,None)
                    event={'schemaVersion':'1.0','type':'device.telemetry','deviceId':device,'bootId':self.boot_id,'messageId':uuid.uuid4().hex,'seq':self.seq,'sentAt':iso(),'payload':payload}
                    async with session.post(url+'/api/device-events/batch',json={'events':[event]},headers=headers) as r:
                        body=await r.json(content_type=None)
                        if r.status>=400:raise RuntimeError(f'telemetry HTTP {r.status}')
                        result=body.get('data',body)
                        if result.get('rejected'):raise RuntimeError('telemetry rejected')
                        if event['messageId'] not in result.get('accepted',[]) and event['messageId'] not in result.get('duplicated',[]):raise RuntimeError('telemetry acknowledgement missing')
                    self.state='online';self.error=None;self.last_success=time.time();backoff=1
                    await asyncio.sleep(self.config['telemetry_interval_s'])
                except Exception as e:
                    self.state='offline';self.error=str(e);registered=None
                    await asyncio.sleep(backoff);backoff=min(backoff*2,30)
