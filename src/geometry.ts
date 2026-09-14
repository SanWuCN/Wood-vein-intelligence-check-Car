import type {MapMeta,Pose} from './types';
export function worldToImage(meta:MapMeta,p:{x:number;y:number}){const a=meta.origin.yaw,dx=p.x-meta.origin.x,dy=p.y-meta.origin.y;return {x:(Math.cos(a)*dx+Math.sin(a)*dy)/meta.resolution,y:meta.height-(-Math.sin(a)*dx+Math.cos(a)*dy)/meta.resolution}}
export function imageToWorld(meta:MapMeta,p:{x:number;y:number}):Pose{const gx=p.x*meta.resolution,gy=(meta.height-p.y)*meta.resolution,a=meta.origin.yaw;return {x:meta.origin.x+Math.cos(a)*gx-Math.sin(a)*gy,y:meta.origin.y+Math.sin(a)*gx+Math.cos(a)*gy,yaw:0}}
