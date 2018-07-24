const net = require('net');

let client = new net.Socket();
client.connect(9876, '192.168.0.1', function () {
  //console.log('Connected');
});

client.on('data', function (data) {
  medicion = (data[0]) | (data[1] << 8) | (data[2] << 16) || (data[3] << 24);
  console.log("Data: " + medicion);
});