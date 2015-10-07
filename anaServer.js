var express = require('express');
var app = express();
var http = require('http').Server(app);
var fs = require('fs');
var path = require('path');
var socket = require('socket.io')(http);
var port = 3000;

app.use(express.static(path.join(__dirname)));

app.get('/', function(req ,res) {
	res.sendfile (__dirname + '/index.html');
});

app.get('/deneme', function(req, res){
	res.sendfile('yeniWeb/index.html');
});

http.listen(port,function(){
	console.log("Listeining: ", port);
});
