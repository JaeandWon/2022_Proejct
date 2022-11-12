const http = require("http");
const fs = require('fs');
const socketio = require('socket.io');
const { response } = require("express");
const PORT = 3000;

const serverbody = (request,response)=>{
    fs.readFile('page.html','utf8',(err,data)=>{
        response.writeHead(200,{'Content-type':'text/html'});
        response.end(data);

        console.log("connected page");
    })
}

const server = http.createServer(serverbody);
const io = require('socket.io')(server);

var kp = 0, ki = 0, kd = 0, gyro = 0;
io.on('connection',client=>{
    console.log('connect!');
    client.on('startmsg',function(data){
        console.log("message(%s)",data);
        io.emit('page',data);
    })
    
})

server.listen(PORT),()=>{
    console.log("Connected to Server");
};