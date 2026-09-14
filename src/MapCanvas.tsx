import {useEffect,useRef,useState} from 'react';
import {Plus,Minus,Maximize,MousePointer2,Navigation} from 'lucide-react';
import type {MapMeta,Pose} from './types';
import {worldToImage,imageToWorld} from './geometry';
type Props={meta:MapMeta|null;pose:Pose|null;scan:number[][];path:Pose[];points:Pose[];tool:'browse'|'pose'|'waypoint';onPlace:(p:Pose)=>void;enabled:boolean};
export function MapCanvas({meta,pose,scan,path,points,tool,onPlace,enabled}:Props){
 const canvas=useRef<HTMLCanvasElement>(null),container=useRef<HTMLDivElement>(null),image=useRef<HTMLImageElement|null>(null);
 const [size,setSize]=useState({width:800,height:500}),[loaded,setLoaded]=useState(0),[view,setView]=useState({scale:1,x:0,y:0}),[draft,setDraft]=useState<{start:Pose;end:Pose}|null>(null);
 const drag=useRef<{screen:{x:number;y:number};view:typeof view;start:Pose;pan:boolean}|null>(null);
 const signature=meta?`${meta.width},${meta.height},${meta.resolution},${meta.origin.x},${meta.origin.y},${meta.origin.yaw}`:'';
 const fit=()=>{if(!meta)return;const b=meta.bounds,pad=34;const scale=Math.min((size.width-pad*2)/Math.max(1,b[2]-b[0]),(size.height-pad*2)/Math.max(1,b[3]-b[1]));setView({scale,x:size.width/2-(b[0]+b[2])/2*scale,y:size.height/2-(b[1]+b[3])/2*scale})};
 useEffect(()=>{const el=container.current;if(!el)return;const ro=new ResizeObserver(([e])=>setSize({width:e.contentRect.width,height:e.contentRect.height}));ro.observe(el);return()=>ro.disconnect()},[]);
 useEffect(()=>{fit()},[signature,size.width,size.height]);
 useEffect(()=>{if(!meta){image.current=null;return}let active=true;const img=new Image();img.onload=()=>{if(active){image.current=img;setLoaded(v=>v+1)}};img.src=`/api/map.png?v=${meta.revision}`;return()=>{active=false}},[meta?.revision,signature]);
 const screen=(p:{x:number;y:number})=>{if(!meta)return {x:0,y:0};const a=worldToImage(meta,p);return {x:a.x*view.scale+view.x,y:a.y*view.scale+view.y}};
 useEffect(()=>{const el=canvas.current;if(!el)return;const c=el.getContext('2d');if(!c)return;const dpr=window.devicePixelRatio||1;el.width=Math.round(size.width*dpr);el.height=Math.round(size.height*dpr);c.scale(dpr,dpr);
 c.fillStyle='#648783';c.fillRect(0,0,size.width,size.height);
 if(!meta||!image.current)return;
 c.imageSmoothingEnabled=false;c.drawImage(image.current,view.x,view.y,meta.width*view.scale,meta.height*view.scale);
 // World-aligned grid remains subtle and shares the same origin rotation as the map.
 c.strokeStyle='rgba(255,255,255,.10)';c.lineWidth=1;
 const meter=view.scale/meta.resolution;
 if(meter>15){const step=Math.ceil(35/meter);for(let i=0;i<meta.width*meta.resolution;i+=step){let x=view.x+i/meta.resolution*view.scale;c.beginPath();c.moveTo(x,0);c.lineTo(x,size.height);c.stroke()}for(let i=0;i<meta.height*meta.resolution;i+=step){let y=view.y+i/meta.resolution*view.scale;c.beginPath();c.moveTo(0,y);c.lineTo(size.width,y);c.stroke()}}
 const line=(ps:Pose[],color:string,dash:number[])=>{if(ps.length<2)return;c.strokeStyle=color;c.lineWidth=2.5;c.setLineDash(dash);c.beginPath();ps.forEach((p,i)=>{const s=screen(p);i?c.lineTo(s.x,s.y):c.moveTo(s.x,s.y)});c.stroke();c.setLineDash([])};
 line(path,'#31d6b8',[]);line(points,'#418cff',[7,6]);
 c.fillStyle='#ff626b';scan.forEach(([x,y])=>{const p=screen({x,y});c.fillRect(p.x-1,p.y-1,2.3,2.3)});
 points.forEach((p,i)=>{const s=screen(p);c.beginPath();c.arc(s.x,s.y,12,0,Math.PI*2);c.fillStyle='#377fff';c.fill();c.strokeStyle='#d9e9ff';c.lineWidth=1.5;c.stroke();c.fillStyle='#fff';c.font='600 12px system-ui';c.textAlign='center';c.textBaseline='middle';c.fillText(String(i+1),s.x,s.y)});
 const arrow=(p:Pose,color:string)=>{const s=screen(p);c.save();c.translate(s.x,s.y);c.rotate(-(p.yaw-meta.origin.yaw));c.fillStyle=color+'30';c.beginPath();c.arc(0,0,22,0,Math.PI*2);c.fill();c.fillStyle=color;c.strokeStyle='#e5ffff';c.lineWidth=1.2;c.beginPath();c.moveTo(14,0);c.lineTo(-9,-8);c.lineTo(-4,0);c.lineTo(-9,8);c.closePath();c.fill();c.stroke();c.restore()};
 if(pose)arrow(pose,'#19cce5');if(draft){const p={...draft.start,yaw:Math.atan2(draft.end.y-draft.start.y,draft.end.x-draft.start.x)};arrow(p,'#ffcf66');line([draft.start,draft.end],'#ffcf66',[])}
 },[meta,pose,scan,path,points,view,size,loaded,draft]);
 const local=(e:React.PointerEvent)=>{const r=canvas.current!.getBoundingClientRect();return {x:e.clientX-r.left,y:e.clientY-r.top}};
 const world=(p:{x:number;y:number})=>imageToWorld(meta!,{x:(p.x-view.x)/view.scale,y:(p.y-view.y)/view.scale});
 const zoom=(ratio:number)=>setView(v=>{const ns=Math.max(.1,Math.min(20,v.scale*ratio)),r=ns/v.scale;return {scale:ns,x:size.width/2+(v.x-size.width/2)*r,y:size.height/2+(v.y-size.height/2)*r}});
 return <div ref={container} className={'map-canvas '+(tool!=='browse'?'placing':'')}>
 <canvas ref={canvas} aria-label="实时地图，拖动设置位置与朝向" onContextMenu={e=>e.preventDefault()} onWheel={e=>zoom(e.deltaY<0?1.12:1/1.12)}
 onPointerDown={e=>{if(!meta)return;const p=local(e),pan=tool==='browse'||e.button!==0;drag.current={screen:p,view:{...view},start:world(p),pan};e.currentTarget.setPointerCapture(e.pointerId);if(!pan&&enabled)setDraft({start:world(p),end:world(p)})}}
 onPointerMove={e=>{const d=drag.current;if(!d||!meta)return;const p=local(e);if(d.pan)setView({...d.view,x:d.view.x+p.x-d.screen.x,y:d.view.y+p.y-d.screen.y});else if(enabled)setDraft({start:d.start,end:world(p)})}}
 onPointerUp={e=>{const d=drag.current;drag.current=null;setDraft(null);if(!d||d.pan||!enabled||!meta)return;const end=world(local(e));const p={...d.start,yaw:Math.hypot(end.x-d.start.x,end.y-d.start.y)>.04?Math.atan2(end.y-d.start.y,end.x-d.start.x):(pose?.yaw||0)};onPlace(p)}} onPointerCancel={()=>{drag.current=null;setDraft(null)}}/>
 {!meta&&<div className="map-empty"><Navigation size={36}/><span>等待地图</span></div>}
 {tool!=='browse'&&<div className="map-mode"><MousePointer2 size={14}/>{tool==='pose'?'人工定位':'添加航点'}</div>}
 <div className="map-zoom"><button aria-label="放大地图" onClick={()=>zoom(1.25)}><Plus/></button><button aria-label="缩小地图" onClick={()=>zoom(.8)}><Minus/></button><button aria-label="适应地图" onClick={fit}><Maximize/></button></div>
 {meta&&<div className="map-scale"><span>1 m</span><i style={{width:Math.min(180,view.scale/meta.resolution)}}/></div>}
 </div>
}
