"use strict";
const assert=require("assert"),Client=require("./perception_client");
class Socket {
  constructor(){this.readyState=0;this.sent=[];Socket.instances.push(this);}
  open(){this.readyState=1;this.onopen();}
  send(raw){this.sent.push(JSON.parse(raw));}
  receive(packet){this.onmessage({data:JSON.stringify(packet)});}
  close(){this.readyState=3;this.onclose();}
}
Socket.instances=[];
const state=(session="s",version=1)=>({session_id:session,state_version:version,header:{stamp:{secs:10,nsecs:0}}});
async function main(){
  const client=new Client("ws://unused","uav1",{WebSocketClass:Socket});
  let socket=client.socket;socket.open();
  socket.receive({op:"service_response",id:"state_get",result:true,values:{success:true,state:state()}});
  const promise=client.mode("track");
  const original=socket.sent.find(x=>x.op==="publish");
  client.disconnect();clearTimeout(client.retryTimer);client.connect();socket=client.socket;socket.open();
  socket.receive({op:"service_response",id:"state_get",result:true,values:{success:true,state:state("s",2)}});
  const replay=socket.sent.find(x=>x.op==="publish");assert.deepStrictEqual(original,replay);
  socket.receive({op:"publish",topic:"/uav1/perception/command_result",msg:{session_id:"s",request_id:original.msg.request_id,stage:"APPLIED",state_version:2}});
  assert.equal((await promise).stage,"APPLIED");assert.equal(client.pending.size,0);
  const rejected=client.model("train4");client.disconnect();clearTimeout(client.retryTimer);client.connect();socket=client.socket;socket.open();
  socket.receive({op:"service_response",id:"state_get",result:true,values:{success:true,state:state("new",1)}});
  assert.equal((await rejected).error_code,"STALE_SESSION");
  assert(!socket.sent.some(x=>x.op==="publish"));
  client.close();console.log("native client reconnect/dedupe/session tests passed");
}
main().catch(error=>{console.error(error);process.exit(1);});
