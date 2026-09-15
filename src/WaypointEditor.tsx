import {useEffect,useRef,useState} from 'react';
import {ArrowDown,ArrowUp,Check,Plus,Trash2,X} from 'lucide-react';
import type {Waypoint} from './types';
type Row={id:number;x:string;y:string};
export function WaypointEditor({points,locked,single,canAdd,onClose,onApply,onAdd}:{points:Waypoint[];locked:boolean;single:boolean;canAdd:boolean;onClose:()=>void;onApply:(p:Waypoint[])=>void;onAdd:(p:Waypoint[])=>void}){
 const ref=useRef<HTMLDialogElement>(null);
 const [rows,setRows]=useState<Row[]>(()=>points.map((p,i)=>({id:i,x:String(Number(p.x.toFixed(3))),y:String(Number(p.y.toFixed(3)))})));
 useEffect(()=>{const dialog=ref.current;dialog?.showModal();return()=>dialog?.close()},[]);
 const invalid=(r:Row)=>[r.x,r.y].some(v=>!v.trim()||!Number.isFinite(Number(v)));
 const valid=rows.every(r=>!invalid(r));
 const values=()=>rows.map(r=>({x:Number(r.x),y:Number(r.y)}));
 const edit=(id:number,k:'x'|'y',value:string)=>setRows(v=>v.map(r=>r.id===id?{...r,[k]:value}:r));
 const move=(i:number,d:number)=>setRows(v=>{const n=[...v];[n[i],n[i+d]]=[n[i+d],n[i]];return n});
 return <dialog ref={ref} className="modal waypoint-editor" aria-labelledby="waypoint-editor-title" onCancel={onClose}>
  <div className="modal-title"><h2 id="waypoint-editor-title">航点编辑 <span>{rows.length} / {single?1:100}</span></h2><button className="icon-button" aria-label="关闭航点编辑" onClick={onClose}><X size={20}/></button></div>
  <div className="waypoint-editor-columns" aria-hidden="true"><span>序号</span><span>X · m</span><span>Y · m</span><span>排序 / 删除</span></div>
  <div className="waypoint-editor-scroll">{rows.length?rows.map((r,i)=><div className="waypoint-editor-row" key={r.id}>
   <span className="waypoint-number">{String(i+1).padStart(2,'0')}</span>
   {(['x','y'] as const).map(k=><label key={k}><span className="wp-mobile-label">{k.toUpperCase()+' · m'}</span><input aria-label={`航点${i+1} ${k.toUpperCase()}`} type="number" step="0.05" value={r[k]} disabled={locked} aria-invalid={invalid(r)} onChange={e=>edit(r.id,k,e.target.value)}/></label>)}
   <div className="editor-row-actions"><button aria-label={`上移航点${i+1}`} disabled={locked||i===0} onClick={()=>move(i,-1)}><ArrowUp size={19}/></button><button aria-label={`下移航点${i+1}`} disabled={locked||i===rows.length-1} onClick={()=>move(i,1)}><ArrowDown size={19}/></button><button aria-label={`删除航点${i+1}`} disabled={locked} onClick={()=>setRows(v=>v.filter(p=>p.id!==r.id))}><Trash2 size={18}/></button></div>
  </div>):<div className="editor-empty"><Plus size={26}/><span>暂无航点</span></div>}</div>
  {!valid&&<div className="editor-validation" role="alert">请输入有效坐标</div>}
  <div className="editor-footer"><button disabled={!canAdd||!valid||rows.length>=100||(single&&rows.length>=1)} onClick={()=>onAdd(values())}><Plus size={18}/>地图选点</button><div><button onClick={onClose}>{locked?'关闭':'取消'}</button><button className="primary" disabled={locked||!valid} onClick={()=>onApply(values())}><Check size={18}/>应用修改</button></div></div>
 </dialog>
}
