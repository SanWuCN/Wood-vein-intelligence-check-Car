import {useEffect,useState} from 'react';
import {request,setToken} from './api';
import type {RobotState} from './types';
export function useRobot(){
 const [state,setState]=useState<RobotState|null>(null),[connected,setConnected]=useState(false),[canControl,setCanControl]=useState(false);
 useEffect(()=>{let closed=false;let socket:WebSocket;let timer:ReturnType<typeof setTimeout>;let stale:ReturnType<typeof setInterval>;let received=0;
 const connect=()=>{if(closed)return;socket=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/api/ws`);
 socket.onmessage=e=>{try{const m=JSON.parse(e.data);if(m.type==='state'){received=Date.now();setState(m.payload);setConnected(true)}}catch{setConnected(false)}};
 socket.onclose=()=>{setConnected(false);if(!closed)timer=setTimeout(connect,1500)};socket.onerror=()=>socket.close()};
 request<{token:string|null;can_control:boolean}>('/session').then(s=>{if(!closed){setToken(s.token||'');setCanControl(s.can_control)}}).catch(()=>setCanControl(false));
 connect();stale=setInterval(()=>{if(Date.now()-received>3000)setConnected(false)},1000);
 return()=>{closed=true;clearTimeout(timer);clearInterval(stale);socket?.close()};},[]);
 return {state,connected,canControl};
}
