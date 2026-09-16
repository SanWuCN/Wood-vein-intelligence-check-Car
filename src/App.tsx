import {useEffect,useRef,useState} from 'react';
import {Map,Route as RouteIcon,Settings,Square,Save,RotateCcw,RotateCw,Magnet,CircleDot,Sparkles,QrCode,Play,Pause,LocateFixed,Trash2,Camera,Radio,Wifi,WifiOff,X,Check,LoaderCircle,Plus,ArrowUp,ArrowDown,ArrowLeft,ArrowRight,Activity,Maximize2,Minimize2,Crosshair} from 'lucide-react';
import {WaypointEditor} from './WaypointEditor';
import {useRobot} from './useRobot';import {request} from './api';import {MapCanvas} from './MapCanvas';
import type {Waypoint,Pose,RefineInfo,SnapInfo,SavedMap,Route,MissionMode,RobotState} from './types';
const fmt=(x:number|null|undefined,n=1)=>x==null||!Number.isFinite(x)?'—':x.toFixed(n);
const duration=(s=0)=>`${Math.floor(s/3600).toString().padStart(2,'0')}:${Math.floor(s/60%60).toString().padStart(2,'0')}:${Math.floor(s%60).toString().padStart(2,'0')}`;
function Video({src,online,title,ratio}:{src:string;online:boolean;title:string;ratio?:number}){const [failed,setFailed]=useState(false);useEffect(()=>setFailed(false),[src,online]);useEffect(()=>{if(online&&failed){const t=setTimeout(()=>setFailed(false),5000);return()=>clearTimeout(t)}},[online,failed]);return <div className="video-pane" style={ratio?{aspectRatio:ratio}:undefined}>{online&&!failed?<img src={src} alt={title} onError={()=>setFailed(true)}/>:<div className="video-empty"><Camera size={26}/><span>{failed?'视频断开':'等待视频'}</span></div>}<span className={'video-live '+(online&&!failed?'on':'')}>{online&&!failed?'LIVE':'OFFLINE'}</span></div>}
function Panel({title,extra,children,className=''}:{title:string;extra?:React.ReactNode;children:React.ReactNode;className?:string}){return <section className={'panel '+className}><div className="panel-heading"><h2>{title}</h2>{extra}</div>{children}</section>}
function Stat({label,value,unit,state}:{label:string;value:string;unit?:string;state?:'online'|'offline'}){return <span className={'stat '+(state||'')}>{label}<b>{value}{unit&&<small>{unit}</small>}</b></span>}
function Metric({label,value,unit}:{label:string;value:string;unit?:string}){return <div className="metric"><span>{label}</span><strong>{value}<small>{unit}</small></strong></div>}
/* IMU 三条曲线：Roll/Pitch/Yaw 分泳道显示，各自按窗口内幅度归一化，
   待命静止时不会因为固定量程被压成一条直线。 */
function Imu({state}:{state:RobotState|null}){
 const i=state?.imu,h=state?.imu_history||[];
 const W=300,H=54,LANES=[{name:'Roll',color:'#518fff',idx:1,min:.6},{name:'Pitch',color:'#43d5b3',idx:2,min:.6},{name:'Yaw',color:'#e7b068',idx:3,min:1.5}],LH=H/LANES.length;
 const series=LANES.map((lane,k)=>{const values=h.map(r=>r[lane.idx]).filter(v=>Number.isFinite(v));let lo=0,hi=1;
  if(values.length){const mn=Math.min(...values),mx=Math.max(...values),span=Math.max(mx-mn,lane.min),mid=(mn+mx)/2;lo=mid-span/2;hi=mid+span/2}
  const top=k*LH,points=h.map((r,j)=>{const v=r[lane.idx];const y=Number.isFinite(v)?top+LH-2-Math.max(0,Math.min(1,(v-lo)/(hi-lo)))*(LH-4):top+LH/2;return `${h.length<2?0:j/(h.length-1)*W},${y}`}).join(' ');
  return {...lane,top,points,last:h.length?h[h.length-1][lane.idx]:null}});
 return <Panel title="IMU" extra={<Activity size={14}/>} className="imu-panel">
  <div className="imu-values">{[['Roll',i?.roll_deg],['Pitch',i?.pitch_deg],['Yaw',i?.yaw_deg]].map(([label,value])=><div key={label as string}><span>{label}</span><strong>{fmt(value as number,2)}<small>°</small></strong></div>)}</div>
  <svg className="imu-chart" viewBox={`0 0 ${W} ${H}`} preserveAspectRatio="none" aria-label="IMU 姿态变化曲线：蓝 Roll、青 Pitch、黄 Yaw">
   {series.map(s=><line key={'base'+s.name} x1="0" y1={s.top+LH-1} x2={W} y2={s.top+LH-1} stroke="#22303f" strokeWidth="1" vectorEffect="non-scaling-stroke"/>)}
   {series.map(s=>s.points.length>3?<polyline key={s.name} fill="none" stroke={s.color} strokeWidth="1.6" strokeLinejoin="round" vectorEffect="non-scaling-stroke" points={s.points}/>:null)}
   {series.map(s=>s.last!=null?<circle key={'dot'+s.name} cx={W-2} cy={s.top+LH/2} r="2" fill={s.color}/>:null)}
   {h.length<2&&<text x={W/2} y={H/2+4} textAnchor="middle" fill="#5d7a99" fontSize="11">等待 IMU 数据</text>}
  </svg></Panel>}
function Modal({title,children,onClose}:{title:string;children:React.ReactNode;onClose:()=>void}){const ref=useRef<HTMLDialogElement>(null);useEffect(()=>{ref.current?.showModal()},[]);return <dialog ref={ref} className="modal" onCancel={onClose} onClick={e=>{if(e.target===ref.current)onClose()}}><div className="modal-title"><h2>{title}</h2><button className="icon-button" aria-label="关闭弹窗" onClick={onClose}><X size={18}/></button></div>{children}</dialog>}
const snapText=(snap?:SnapInfo|null)=>{
 if(!snap)return '定位已提交';
 const parts:string[]=[];
 if(snap.free_shift_m>0.004)parts.push(`已移出障碍 ${snap.free_shift_m.toFixed(2)} m`);
 if(snap.shift_m>0.004||Math.abs(snap.shift_deg)>=0.1)parts.push(`已吸附微调 ${snap.shift_m.toFixed(2)} m / ${snap.shift_deg.toFixed(1)}°`);
 if(parts.length)return '定位已提交 · '+parts.join(' · ');
 if(snap.reason==='weak_match')return '定位已提交（未吸附：此处吻合度不足）';
 if(snap.reason==='no_scan')return '定位已提交（未吸附：无雷达数据）';
 if(snap.reason==='no_map')return '定位已提交（未吸附：地图未就绪）';
 return '定位已提交（已在最佳位置）';
};
const refineText=(r?:RefineInfo|null)=>{
 if(!r||!r.enabled)return '静止校准已关闭';
 if(r.applied&&r.shift_m!=null)return `静止校准 · 已校正 ${r.shift_m.toFixed(2)} m / ${(r.shift_deg??0).toFixed(1)}°`;
 if(r.reason==='settling')return `静止校准 · 静止 ${r.idle_s.toFixed(0)} s，待稳定`;
 if(r.reason==='wait')return '静止校准 · 等待下次校准';
 if(r.reason==='match_low')return '静止校准 · 吻合度偏低，暂不校准';
 if(r.reason==='waiting_data')return '静止校准 · 等待底盘数据';
 if(r.reason==='moving')return '静止校准 · 小车移动中暂停';
 if(r.reason==='mission')return '静止校准 · 任务进行中暂停';
 if(r.reason==='no_gain')return '静止校准 · 当前已对齐';
 if(r.reason==='error')return '静止校准 · 本次失败，稍后重试';
 return '静止校准已开启';
};
const missionNames:Record<string,string>={idle:'待命',accepting:'任务下发中',running:'巡航中',paused:'已暂停',pausing:'暂停中',stopping:'停止中',stopped:'已停止',completed:'已完成',failed:'任务失败'};
export function App(){
 const initial=new URLSearchParams(location.search),initialTab=initial.get('tab')==='cruise'?'cruise':'mapping';
 const {state:s,connected,canControl}=useRobot();const [tab,setTab]=useState<'mapping'|'cruise'>(initialTab),[mapView,setMapView]=useState<'map'|'rviz'>(initial.get('view')==='rviz'?'rviz':initial.get('view')==='map'?'map':initialTab==='cruise'?'map':'rviz'),[tool,setTool]=useState<'browse'|'pose'|'waypoint'>('browse'),[expanded,setExpanded]=useState(false);
 const [maps,setMaps]=useState<SavedMap[]>([]),[routes,setRoutes]=useState<Route[]>([]),[selectedMap,setSelectedMap]=useState(''),[points,setPoints]=useState<Waypoint[]>([]),[mode,setMode]=useState<MissionMode>('multi'),[speed,setSpeed]=useState(.2);
 const [busy,setBusy]=useState(''),[toast,setToast]=useState<{message:string;error:boolean}|null>(null),[modal,setModal]=useState<'save'|'restart'|'settings'|'route'|'load'|'waypoints'|'reset'|null>(null),[name,setName]=useState(''),[clock,setClock]=useState(new Date());
 const [dismissedError,setDismissedError]=useState(''),[snap,setSnap]=useState<SnapInfo|null>(null),[pathStale,setPathStale]=useState(false);
 const [config,setConfig]=useState<Record<string,string>>({}),[tokenEdited,setTokenEdited]=useState(false),[routeId,setRouteId]=useState('');
 const actionAllowed=connected&&canControl&&!busy&&!s?.transition;const running=['running','accepting','pausing','stopping'].includes(s?.mission.state||'');const locked=running||s?.mission.state==='paused';
 const mapLoaded=s?.mode==='navigation'&&!!s?.active_map_id&&s.active_map_id===selectedMap;const currentMap=maps.find(m=>m.id===s?.active_map_id);
 const refresh=async()=>{const [m,r]=await Promise.all([request<{maps:SavedMap[]}>('/maps'),request<{routes:Route[]}>('/routes')]);setMaps(m.maps);setRoutes(r.routes);setSelectedMap(v=>v||s?.active_map_id||m.maps[0]?.id||'')};
 useEffect(()=>{refresh().catch(()=>{});const t=setInterval(()=>setClock(new Date()),1000);return()=>clearInterval(t)},[]);
 /* 部署新版本后自动重载：对比当前页面脚本名与后端 dist 的构建号（任务状态在服务端，重载不影响运行中的任务） */
 useEffect(()=>{
  const loaded=(document.querySelector('script[type=module][src]') as HTMLScriptElement|null)?.src.split('/').pop()||'';
  const timer=setInterval(async()=>{try{
   const r=await fetch('/api/health',{cache:'no-store'});const j=await r.json();
   if(j?.build&&loaded&&!loaded.includes(j.build)){console.info('控制台已更新，正在重载…',j.build);location.reload()}
  }catch{}},30000);
  return()=>clearInterval(timer);
 },[]);
 useEffect(()=>{if(s?.active_map_id)setSelectedMap(s.active_map_id)},[s?.active_map_id]);
 useEffect(()=>{if(s?.mission&&['accepting','running','paused','pausing'].includes(s.mission.state)){setPoints(v=>JSON.stringify(v)===JSON.stringify(s.mission.points)?v:s.mission.points);setMode(s.mission.mode)}},[s?.mission]);
 useEffect(()=>{if(s?.speed_mps)setSpeed(s.speed_mps)},[s?.speed_mps]);
 useEffect(()=>{if(toast){const t=setTimeout(()=>setToast(null),5000);return()=>clearTimeout(t)}},[toast]);
 const run=async(label:string,fn:()=>Promise<unknown>,message?:string)=>{setBusy(label);try{await fn();if(message)setToast({message,error:false});return true}catch(e){setToast({message:(e as Error).message,error:true});return false}finally{setBusy('')}};
 const changeTab=(t:'mapping'|'cruise')=>{setTab(t);setTool('browse');setMapView(t==='cruise'?'map':'rviz');refresh().catch(()=>{})};
 const place=async(p:Pose)=>{if(!actionAllowed)return;
  if(tool==='pose'){
   setTool('browse');setBusy('localize');
   try{const r=await request<{snap?:SnapInfo|null}>('/navigation/localize',{pose:p});setSnap(r.snap||null);setToast({message:snapText(r.snap),error:false})}
   catch(e){setToast({message:(e as Error).message,error:true})}
   finally{setBusy('')}
  }else if(tool==='waypoint'){setPoints(v=>mode==='single'?[{x:p.x,y:p.y}]:v.length<100?[...v,{x:p.x,y:p.y}]:v);setRouteId('')}};
 /* 微调：在地图视图里按屏幕方向平移 2 cm / 旋转 1.5°，不触发吸附，便于消除残余偏差 */
 const nudge=(right:number,up:number,turn:number)=>{
  const pose=s?.pose;if(!pose||!actionAllowed||locked||busy)return;
  const a=s?.map?.origin.yaw||0,step=.02,rot=1.5*Math.PI/180;
  const wx=(right*Math.cos(a)+up*Math.sin(a))*step,wy=(right*Math.sin(a)-up*Math.cos(a))*step;
  run('nudge',()=>request('/navigation/localize',{pose:{x:pose.x+wx,y:pose.y+wy,yaw:pose.yaw+turn*rot},snap:false}),
      turn?`已微调 ${Math.abs(turn*1.5).toFixed(1)}°（${turn>0?'顺时针':'逆时针'}）`:`已微调 ${Math.round(Math.hypot(wx,wy)*100)} cm`);
 };
 const openSettings=()=>run('settings',async()=>{const c=await request<Record<string,string>>('/config');setConfig({...c,platform_token:''});setTokenEdited(false);setModal('settings')});
 const loadMap=(saveCurrent?:boolean)=>run('load',async()=>{
   const r=await request<{backup?:{name:string}|null}>('/navigation/load',{map_id:selectedMap,...(saveCurrent===undefined?{}:{save_current:saveCurrent})});
   setPoints([]);setRouteId('');setTool('browse');setMapView('map');setModal(null);await refresh();
   setToast({message:saveCurrent===false?'已切换地图（未保存当前建图）':r.backup?`地图已加载，当前建图已备份为「${r.backup.name}」`:'地图已加载（当前建图此前已保存，未重复备份）',error:false});
 },'');
 const chooseMode=(m:MissionMode)=>{setMode(m);if(m==='single')setPoints(v=>v.slice(0,1))};
 const refine=s?.localization?.refine;
 const toggleRefine=()=>run('refine',()=>request('/navigation/refine',{enabled:!refine?.enabled}),refine?.enabled?'已关闭静止校准':'已开启静止校准');
 /* 静止校准状态提示：开启时持续显示，刚校正过或有关键提示时也显示 */
 const relocalize=s?.localization?.relocalize;
 const aruco=s?.localization?.aruco;
 const captureMarker=()=>run('aruco',async()=>{const r=await request<{marker_pose:{x:number;y:number}}>('/aruco/capture',{});setToast({message:`已记录标签 #${aruco?.marker_id??''} 的地图位置 (${r.marker_pose.x.toFixed(2)}, ${r.marker_pose.y.toFixed(2)})，相机校正已启用`,error:false})},'');
 const refineHint=refine&&(refine.enabled||refine.applied)?refineText(refine):'';
 const pointsSig=JSON.stringify(points);
 useEffect(()=>{setPathStale(true)},[pointsSig]);   // 航点一改，旧路径就不再对应当前路线
 const record=s?.record;
 const recordedPoints:Waypoint[]=(record?.points||[]).map(([x,y])=>({x,y}));
 /* 录制/手绘的原始轨迹含定位抖动，局部转弯可能比车的最小转弯半径还紧，先整理成可行驶序列 */
 const tidyPoints=async(raw:Waypoint[],label:string)=>{const r=await request<{points:Waypoint[];raw_count:number;count:number;min_radius:number|null}>('/routes/simplify',{points:raw});return {points:r.points,note:`${label}：${r.raw_count} → ${r.count} 个航点，最小转弯半径 ${r.min_radius!=null?r.min_radius.toFixed(2)+' m':'—'}（车 0.35 m）`}};
 const applyRecorded=()=>run('tidy',async()=>{if(!recordedPoints.length)return;
   const {points:clean,note}=await tidyPoints(recordedPoints,'已整理录制轨迹');
   setPoints(clean);setRouteId('');setToast({message:note+'，可直接保存为路线',error:false})});
 const tidyCurrent=()=>run('tidy',async()=>{if(points.length<2)return;
   const {points:clean,note}=await tidyPoints(points,'已整理当前航点');
   setPoints(clean);setRouteId('');setToast({message:note,error:false})});
 const toggleRecord=()=>run('record',()=>request('/record/'+(record?.active?'stop':'start'),{}),record?.active?`已停止记录（${record?.count||0} 点 / ${(record?.distance_m||0).toFixed(1)} m）`:'开始记录航迹，遥控小车行驶即可');
 const rvizHint=!connected?'未连接':s?.mode==='mapping'?'建图中':s?.mode==='navigation'?(running?missionNames[s?.mission.state||'idle']:!s?.navigation?.ready?'导航未就绪':!s?.localization.ready?'待定位':'已定位 · 待命'):'待命 · 开始建图后显示实时地图';
 const mapTools=<div className="segmented compact">{[['browse','浏览'],['pose','定位'],['waypoint','航点']].map(([t,label])=><button key={t} className={tool===t?'active':''} title={t==='pose'?'人工定位：在地图上点击设置初始位姿':t==='waypoint'?'在地图上点击添加航点':undefined} disabled={t!=='browse'&&(!actionAllowed||!mapLoaded||locked)} onClick={()=>setTool(t as typeof tool)}>{label}</button>)}</div>;
 return <div className={'console-shell'+(expanded?' map-expanded':'')}>
 <header className="topbar"><img className="brand" src="/brand/mumai-wordmark-white.png" alt="木脉智检"/><nav aria-label="主要功能"><button className={tab==='mapping'?'active':''} onClick={()=>changeTab('mapping')}><Map size={17}/>建图</button><button className={tab==='cruise'?'active':''} onClick={()=>changeTab('cruise')}><RouteIcon size={17}/>巡航</button></nav><div className="top-status">{s?.simulated&&<span className="demo-label">演示</span>}<span className={'connection '+(connected?'online':'')}><i/>{connected?(canControl?'已连接':'只读'):'未连接'}</span><button className="icon-button" aria-label="连接设置" title="连接设置" disabled={!canControl} onClick={openSettings}><Settings size={17}/></button><button aria-label="环境重置" title="环境重置" disabled={!actionAllowed} onClick={()=>setModal('reset')}><RotateCcw size={15}/><span>重置</span></button><button className="stop-button" onClick={()=>run('stop',()=>request('/control/stop',{}),'已停止')} disabled={!connected||!canControl}><Square size={12} fill="currentColor"/>停止</button></div></header>
 <main className="workspace"><div className="primary-column">
 <div className="page-toolbar">{tab==='mapping'?<><h1>{s?.mode==='mapping'?'实时建图':'地图工作台'}</h1><div className="toolbar-stats"><Stat label="扫描频率" value={connected?fmt(s?.lidar.hz):'—'} unit="Hz"/><Stat label="地图分辨率" value={fmt(s?.map?.resolution,2)} unit="m"/><Stat label="已运行" value={duration(s?.uptime_s)}/></div><div className="toolbar-actions">{s?.mode==='mapping'?<button disabled={!actionAllowed} onClick={()=>setModal('restart')}><RotateCcw size={15}/>重置地图</button>:<button className="primary" disabled={!actionAllowed||locked} onClick={()=>run('start',()=>request('/mapping/start',{}),'建图已启动')}><Play size={15}/>开始建图</button>}<button className="primary" disabled={!actionAllowed||!s?.map} onClick={()=>{setName('地图 '+clock.toLocaleString('zh-CN',{month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit'}));setModal('save')}}><Save size={15}/>保存地图</button></div></>:<><div className="map-select-wrap"><select aria-label="选择地图" value={selectedMap} onChange={e=>{setSelectedMap(e.target.value);setPoints([]);setRouteId('')}} disabled={locked||!!busy}><option value="">选择地图</option>{maps.map(m=><option key={m.id} value={m.id}>{m.name}</option>)}</select><button className="primary" disabled={!selectedMap||!actionAllowed||locked} onClick={()=>s?.mode==='mapping'?(s?.mapping?.changed===false?loadMap():setModal('load')):loadMap()}>{busy==='load'?<LoaderCircle className="spin" size={15}/>:<Map size={15}/>}加载地图</button></div><div className="toolbar-stats"><Stat label="定位" value={`${s?.localization.ready?'已定位':'待定位'}${s?.localization.match!=null?' '+fmt(s.localization.match*100,0)+'%':''}`} state={s?.localization.ready?'online':'offline'}/><Stat label="X" value={fmt(s?.pose?.x,2)} unit="m"/><Stat label="Y" value={fmt(s?.pose?.y,2)} unit="m"/><Stat label="Yaw" value={fmt(s?.pose?s.pose.yaw*180/Math.PI:null,1)} unit="°"/></div><div className="toolbar-actions"><button disabled={!actionAllowed||!mapLoaded||locked} onClick={()=>run('auto-localize',()=>request('/navigation/auto-localize',{}),'正在定位')}><Crosshair size={15}/>自动定位</button></div></>}</div>
 <Panel title={tab==='mapping'?'实时地图':'巡航地图'} className="map-panel" extra={<div className="map-head-tools">{tab==='cruise'&&mapView==='map'&&mapTools}{tab==='cruise'&&mapView==='map'&&<span className={'localization-state '+(s?.localization.ready?'online':'')}>{s?.mode==='navigation'&&!s?.navigation?.ready?'导航初始化':s?.localization.ready?'已定位':s?.mode==='navigation'?'待定位':'未加载'}</span>}<div className="segmented compact"><button className={mapView==='map'?'active':''} onClick={()=>setMapView('map')}>地图</button><button className={mapView==='rviz'?'active':''} onClick={()=>{setMapView('rviz');setTool('browse')}}>RViz</button></div><button className={'icon-button'+(aruco?.seeing?' on':'')} aria-label="记录标签位置" title={aruco?(aruco.configured?`标签 #${aruco.marker_id} 已标定${aruco.seeing?'（当前可见）':'（当前看不到）'}；再点一次可重新标定`:'把车对准标签后点这里标定标签地图位置'):'相机标签定位未启用'} disabled={!actionAllowed||!mapLoaded||!aruco?.seeing||(s?.localization.match??0)<.8} onClick={captureMarker}><QrCode size={15}/></button><button className={'icon-button'+(refine?.enabled?' on':'')} aria-label={refine?.enabled?'关闭静止校准':'开启静止校准'} title={(refine?.enabled?'关闭':'开启')+'静止校准：定位准后底盘静止时周期性吸附校正'} disabled={!actionAllowed||s?.mode!=='navigation'} onClick={toggleRefine}><Magnet size={15}/></button><button className="icon-button" aria-label={expanded?'退出全屏地图':'全屏地图'} title={expanded?'退出全屏地图':'全屏地图'} onClick={()=>setExpanded(v=>!v)}>{expanded?<Minimize2 size={15}/>:<Maximize2 size={15}/>}</button></div>}>
 <div className="map-view">
 {mapView==='map'?<><MapCanvas meta={s?.map||null} pose={s?.pose||null} scan={s?.scan_points||[]} path={pathStale?[]:(s?.path||[])} points={tab==='cruise'?points:[]} record={tab==='cruise'?recordedPoints:[]} tool={tool} onPlace={place} enabled={actionAllowed&&mapLoaded&&!locked}/>{tab==='cruise'&&mapLoaded&&!locked&&<div className="map-nudge" role="group" aria-label="定位微调">{[[<RotateCcw size={13}/>,'逆时针 1.5°',0,0,-1,'逆时针微调'],[<ArrowUp size={13}/>,'向上 2 cm',0,1,0,'向上微调'],[<RotateCw size={13}/>,'顺时针 1.5°',0,0,1,'顺时针微调'],[<ArrowLeft size={13}/>,'向左 2 cm',-1,0,0,'向左微调'],[<ArrowDown size={13}/>,'向下 2 cm',0,-1,0,'向下微调'],[<ArrowRight size={13}/>,'向右 2 cm',1,0,0,'向右微调']].map(([icon,title,r,u,turn,label],i)=><button key={i} title={title as string} aria-label={label as string} disabled={!actionAllowed||busy==='nudge'} onClick={()=>nudge(r as number,u as number,turn as number)}>{icon}</button>)}</div>}</>:<><div className="rviz-view"><Video src="/api/streams/rviz.mjpeg" online={connected&&s?.streams.rviz_state==='online'} title="RViz 实时画面"/></div><span className="map-hint"><Radio size={12}/>{rvizHint}</span></>}
 {tab==='cruise'&&mapView==='map'&&refineHint&&<span className="map-hint refine-hint"><Magnet size={12}/>{refineHint}</span>}
 {tab==='cruise'&&mapView==='map'&&!!aruco?.count&&<span className="map-hint refine-hint" style={{right:8,top:68}}><QrCode size={12}/>相机标签校正 {aruco.count} 次{aruco.shift_m!=null?`（最近 ${aruco.shift_m.toFixed(2)} m / ${(aruco.shift_deg??0).toFixed(1)}°）`:''}</span>}
 {tab==='cruise'&&mapView==='map'&&!!relocalize?.count&&(relocalize.count>0)&&<span className={'map-hint '+(relocalize.reason==='stopped'?'stale-hint':'refine-hint')} style={{right:8,top:38}}><Crosshair size={12}/>航行中自动校准 {relocalize.count} 次{relocalize.shift_m!=null?`（最近 ${relocalize.shift_m.toFixed(2)} m / ${(relocalize.shift_deg??0).toFixed(1)}°）`:''}{relocalize.reason==='stopped'?' · 已安全停车':''}</span>}
 {tab==='cruise'&&mapView==='map'&&record?.active&&<span className="map-hint record-hint"><CircleDot size={12}/>记录中 {record.count} 点 / {(record.distance_m||0).toFixed(1)} m</span>}
 {tab==='cruise'&&mapView==='map'&&!record?.active&&pathStale&&points.length>0&&(s?.path?.length||0)>0&&<span className="map-hint stale-hint">航点已改动，路径未更新，请重新“预览”</span>}
 {s?.transition&&<div className="map-loading"><LoaderCircle className="spin" size={22}/><span>{s.transition==='reset'?'正在重置环境':s.transition==='mapping'?'正在启动建图':'正在加载地图'}</span></div>}
 </div>
 </Panel>
 </div><aside className="inspector">
 <Panel title="现场视频" className="camera-panel" extra={<span className={'micro-state '+(connected&&s?.camera.state==='online'?'online':'')}>{connected&&s?.camera.state==='online'?`${fmt(s.camera.fps,0)} FPS`:'离线'}</span>}><Video src="/api/streams/camera.mjpeg" online={connected&&s?.camera.state==='online'} title="现场摄像头" ratio={s?.camera.size?s.camera.size.width/s.camera.size.height:4/3}/></Panel>
 {tab==='mapping'?<>
 <Imu state={connected?s:null}/>
 <Panel title="设备状态" className="device-panel"><div className="telemetry-grid"><Metric label="速度" value={connected?fmt(s?.velocity?.linear_mps,2):'—'} unit="m/s"/><Metric label="CPU" value={connected?fmt(s?.metrics.cpu_percent,0):'—'} unit="%"/><Metric label="温度" value={connected?fmt(s?.metrics.temperature_c,1):'—'} unit="°C"/><Metric label="内存" value={connected?fmt(s?.metrics.memory_percent,0):'—'} unit="%"/><Metric label="雷达" value={connected?fmt(s?.lidar.hz,1):'—'} unit="Hz"/><Metric label="角速度" value={connected?fmt(s?.velocity?.angular_rps,2):'—'} unit="rad/s"/><Metric label={s?.battery?.percentage!=null?'电量':'电池'} value={connected?fmt(s?.battery?.percentage??s?.battery?.voltage_v,s?.battery?.percentage!=null?0:2):'—'} unit={s?.battery?.percentage!=null?'%':'V'}/><Metric label="底盘" value={connected&&s?.chassis?.state==='online'?'在线':'离线'}/></div><div className="chassis-detail"><span>阿克曼底盘</span><span>X {fmt(s?.chassis?.odometry?.pose.x,2)} · Y {fmt(s?.chassis?.odometry?.pose.y,2)} m</span><span>{s?.battery?.charging===true?'充电中':s?.battery?.charging===false?'未充电':'—'}</span></div></Panel></>
 :<>
 <Panel title="巡航任务" className="mission-panel" extra={<span className={'micro-state '+(running?'online':'')}>{missionNames[s?.mission.state||'idle']}</span>}>
 <div className="mission-body"><div className="segmented">{([['single','点到点'],['multi','多点'],['loop','循环']] as const).map(([m,l])=><button key={m} className={mode===m?'active':''} disabled={locked} onClick={()=>chooseMode(m)}>{l}</button>)}</div>
 <div className="route-tools"><select aria-label="选择已保存路线" value={routeId} disabled={locked} onChange={e=>{const r=routes.find(r=>r.id===e.target.value);setRouteId(e.target.value);if(r){setPoints(r.points);setMode(r.mode);setSpeed(r.speed_mps)}}}><option value="">新建路线</option>{routes.filter(r=>r.map_id===selectedMap).map(r=><option key={r.id} value={r.id}>{r.name}</option>)}</select><button className="icon-button" aria-label="保存路线" title="保存路线" disabled={!points.length||!selectedMap||!actionAllowed||locked} onClick={()=>{setName('巡航路线');setModal('route')}}><Save size={15}/></button><button className="icon-button" aria-label="清空航点" title="清空航点" disabled={!points.length||locked} onClick={()=>setPoints([])}><Trash2 size={15}/></button></div>
 <div className="waypoint-tools"><button className={'record-button'+(record?.active?' active':'')} aria-label={record?.active?'停止记录航迹':'开始记录航迹'} title={(record?.active?'停止记录':'记录航迹')+'：遥控行驶时每 20 cm 记一个点（重新开始会清空上次记录）'} disabled={!actionAllowed||!s?.pose} onClick={toggleRecord}><CircleDot size={14}/>{record?.active?`停止记录 ${record.count} 点 / ${(record.distance_m||0).toFixed(1)} m`:'记录航迹'}</button><button className="record-button" aria-label="用记录的点作为航点" title={`把记录的 ${record?.count||0} 个点整理后填入航点列表，再点保存路线`} disabled={!recordedPoints.length||locked} onClick={applyRecorded}><RouteIcon size={14}/>用为航点</button><button className="record-button" aria-label="整理当前航点" title="平滑+抽稀当前航点，去掉定位抖动造成的过紧转弯（10cm 录制轨迹尤其需要）" disabled={points.length<2||locked} onClick={tidyCurrent}><Sparkles size={14}/>整理航点</button></div>
 <button className="waypoint-preview" aria-label={`编辑航点，共 ${points.length} 个`} onClick={()=>setModal('waypoints')}><span className="waypoint-preview-head"><strong>航点 · {points.length}</strong><span>查看 / 编辑 <Maximize2 size={13}/></span></span><span className="waypoint-preview-list">{points.length?points.map((p,i)=><span className="waypoint-preview-row" key={i}><span className="waypoint-number">{i+1}</span><span>X {p.x.toFixed(2)}</span><span>Y {p.y.toFixed(2)}</span></span>):<span className="waypoint-preview-empty"><Plus size={20}/>添加航点</span>}</span></button>
 <div className="speed-control"><label htmlFor="speed">巡航速度</label><input id="speed" type="range" min="0.05" max={s?.max_speed_mps||.35} step="0.01" value={speed} disabled={!actionAllowed} onChange={e=>setSpeed(Number(e.target.value))} onPointerUp={()=>run('speed',()=>request('/navigation/speed',{speed_mps:speed}))} onKeyUp={e=>{if(e.key.startsWith('Arrow'))run('speed',()=>request('/navigation/speed',{speed_mps:speed}))}}/><strong>{speed.toFixed(2)}<small>m/s</small></strong></div>
 <div className="mission-actions"><button className="preview-button" disabled={!actionAllowed||!mapLoaded||!s?.localization.ready||!s?.navigation?.planner_ready||!points.length||locked} onClick={()=>run('preview',async()=>{const r=await request<{tight_turn_ratio?:number;min_radius?:number|null}>('/navigation/preview',{points,mode});setPathStale(false);setToast({message:r.tight_turn_ratio&&r.tight_turn_ratio>0?`路径已生成；但路线有 ${Math.round(r.tight_turn_ratio*100)}% 的转弯比车的最小转弯半径(0.35m)更紧，规划可能绕行——建议点“整理航点”`:'路径已生成',error:false})},'')}><RouteIcon size={14}/>{busy==='preview'?'规划中':'预览'}</button><button className="primary" disabled={!actionAllowed||!mapLoaded||!s?.localization.ready||!s?.navigation?.ready||!points.length||(mode==='loop'&&points.length<2)||running} onClick={()=>run('mission',()=>s?.mission.state==='paused'?request('/navigation/resume',{}):request('/navigation/start',{map_id:selectedMap,points,mode,speed_mps:speed}))}><Play size={15} fill="currentColor"/>{s?.mission.state==='paused'?'继续':'开始巡航'}</button><button disabled={!actionAllowed||!running} onClick={()=>run('pause',()=>request('/navigation/pause',{}))}><Pause size={15}/>暂停</button></div>
 {running&&<div className="mission-progress"><span>航点 {(s?.mission.index||0)+1} / {s?.mission.points.length}</span>{!!s?.mission.skipped&&<span>已跳过 {s.mission.skipped} 个点{s?.mission.skip_reason?`（${s.mission.skip_reason}）`:''}</span>}<span>{mode==='loop'?`第 ${(s?.mission.cycle||0)+1} 圈`:fmt(s?.mission.distance_remaining,1)+' m'}</span></div>}
 </div></Panel></>}
 </aside></main>
 <footer className="statusbar"><div><span className={'connection '+(connected?'online':'')}><i/>{s?.device_id||'小车'}</span><span className="footer-divider"/><span>{tab==='cruise'?(currentMap?.name||'未加载地图'):(s?.mode==='mapping'?'建图中':'待命')}</span></div><div><span className={'platform-state '+(s?.platform.state==='online'?'online':'')}>{s?.platform.state==='online'?<Wifi size={12}/>:<WifiOff size={12}/>}平台{s?.platform.state==='online'?'已连接':s?.platform.state==='unconfigured'?'未配置':'未连接'}</span><span className="footer-divider"/><time>{clock.toLocaleTimeString('zh-CN',{hour12:false})}</time></div></footer>
 {(toast||((s?.last_error||s?.error)&&dismissedError!==(s?.last_error||s?.error)))&&<div role="status" className={'toast '+((toast?.error||s?.error||s?.last_error)?'error':'')}><span>{toast?.message||s?.last_error||s?.error}</span><button aria-label="关闭提示" onClick={()=>{setToast(null);setDismissedError(s?.last_error||s?.error||'')}}><X size={15}/></button></div>}
 {modal==='waypoints'&&<WaypointEditor points={points} locked={locked||!actionAllowed} single={mode==='single'} canAdd={actionAllowed&&mapLoaded&&!locked} onClose={()=>setModal(null)} onApply={next=>{setPoints(next);setRouteId('');setModal(null)}} onAdd={next=>{setPoints(next);setRouteId('');setModal(null);setMapView('map');setTool('waypoint')}}/>}
 {modal==='reset'&&<Modal title="环境重置" onClose={()=>!busy&&setModal(null)}><p className="confirm-line">停止任务并重启底盘与定位环境。保留已保存地图和路线，当前建图自动备份。</p><div className="modal-actions"><button disabled={!!busy} onClick={()=>setModal(null)}>取消</button><button className="primary" disabled={!actionAllowed} onClick={()=>run('reset',async()=>{await request('/control/reset',{});setPoints([]);setRouteId('');setTool('browse');setModal(null);await refresh()},'环境已重置，请重新加载地图')}>{busy==='reset'?<LoaderCircle className="spin" size={16}/>:<RotateCcw size={16}/>}重置环境</button></div></Modal>}
 {modal==='save'&&<Modal title="保存地图" onClose={()=>setModal(null)}><form onSubmit={e=>{e.preventDefault();run('save',async()=>{await request('/mapping/save',{name});setModal(null);await refresh()},'地图已保存')}}><label>地图名称<input autoFocus maxLength={40} required value={name} onChange={e=>setName(e.target.value)}/></label><div className="modal-actions"><button type="button" onClick={()=>setModal(null)}>取消</button><button className="primary" disabled={!!busy}><Save size={15}/>保存地图</button></div></form></Modal>}
 {modal==='route'&&<Modal title="保存路线" onClose={()=>setModal(null)}><form onSubmit={e=>{e.preventDefault();run('save-route',async()=>{await request('/routes/save',{name,map_id:selectedMap,points,mode,speed_mps:speed});setModal(null);await refresh()},'路线已保存')}}><label>路线名称<input autoFocus required maxLength={40} value={name} onChange={e=>setName(e.target.value)}/></label><div className="modal-actions"><button type="button" onClick={()=>setModal(null)}>取消</button><button className="primary" disabled={!!busy}>保存路线</button></div></form></Modal>}
 {modal==='load'&&<Modal title="切换到巡航" onClose={()=>setModal(null)}><p className="confirm-line">{s?.mapping?.saved?`当前建图相对上次保存「${s.mapping.saved_name||''}」有改动（差异约 ${((s.mapping.diff_ratio||0)*100).toFixed(1)}%）。`:'当前建图还没有保存过。'}<br/>切换前要不要先保存一份？</p><div className="modal-actions"><button onClick={()=>setModal(null)}>取消</button><button disabled={!!busy} onClick={()=>loadMap(false)}>不保存，直接切换</button><button className="primary" disabled={!!busy} onClick={()=>loadMap(true)}>{busy==='load'?<LoaderCircle className="spin" size={15}/>:<Save size={15}/>}保存并切换</button></div></Modal>}
 {modal==='restart'&&<Modal title="重置地图" onClose={()=>setModal(null)}><p className="confirm-line">当前地图将自动保存</p><div className="modal-actions"><button onClick={()=>setModal(null)}>取消</button><button className="primary" disabled={!!busy} onClick={()=>run('restart',async()=>{await request('/mapping/restart',{});setModal(null);await refresh()},'已重置地图')}>{busy?<LoaderCircle className="spin" size={15}/>:<RotateCcw size={15}/>}确定</button></div></Modal>}
 {modal==='settings'&&<Modal title="连接设置" onClose={()=>setModal(null)}><form onSubmit={e=>{e.preventDefault();const b={platform_url:config.platform_url,rviz_rtmp_url:config.rviz_rtmp_url,camera_rtmp_url:config.camera_rtmp_url,...(tokenEdited?{platform_token:config.platform_token}:{})};run('settings-save',async()=>{await request('/settings/save',b);setModal(null)},'设置已保存')}}><label>设备编号<input value={config.device_id||''} readOnly/></label><label>平台地址<input type="url" placeholder="https://platform.example.com" value={config.platform_url||''} onChange={e=>setConfig(v=>({...v,platform_url:e.target.value}))}/></label><label>设备令牌<input type="password" autoComplete="new-password" placeholder={config.platform_token_set?'已设置':'未设置'} value={config.platform_token||''} onChange={e=>{setTokenEdited(true);setConfig(v=>({...v,platform_token:e.target.value}))}}/></label><label>RViz · RTMP<input placeholder="rtmp://host/live/rviz" value={config.rviz_rtmp_url||''} onChange={e=>setConfig(v=>({...v,rviz_rtmp_url:e.target.value}))}/></label><label>摄像头 · RTMP<input placeholder="rtmp://host/live/camera" value={config.camera_rtmp_url||''} onChange={e=>setConfig(v=>({...v,camera_rtmp_url:e.target.value}))}/></label><div className="modal-actions"><button type="button" onClick={()=>setModal(null)}>取消</button><button className="primary" disabled={!!busy}><Check size={15}/>保存设置</button></div></form></Modal>}
 </div>
}
