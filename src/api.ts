let controlToken='';
export const setToken=(t:string)=>{controlToken=t};
export async function request<T=Record<string,unknown>>(path:string,body?:unknown):Promise<T>{
 const response=await fetch('/api'+path,{method:body===undefined?'GET':'POST',headers:{'Content-Type':'application/json',...(controlToken?{'X-Control-Token':controlToken}:{}),...(body!==undefined?{'X-Request-Id':crypto.randomUUID()}: {})},body:body===undefined?undefined:JSON.stringify(body)});
 const result=await response.json();if(!response.ok||result.ok===false)throw new Error(result.error?.message||'请求失败');return result;
}
