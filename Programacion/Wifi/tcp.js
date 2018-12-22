const net = require('net');
const readline = require('readline');

const rl = readline.createInterface({
  input: process.stdin,
  output: process.stdout
});

let client = new net.Socket();
client.connect(9876, '192.168.0.1', function () {
  //console.log('Connected');
});

client.on('data', function (data) {
  medicion = (data[0]) | (data[1] << 8) | (data[2] << 16) || (data[3] << 24);
  console.log(dictionary[parseInt(state)] + medicion);
});

let state = '1';
let dictionary = {
  1: "Led 1: ",
  3: "Led 3: ",
  4: "Led 4: ",
  6: "Led 6: ",
  7: "Ángulo: ",
  8: "Laberinto: "
};

function loop() {
  rl.question('> ', (answer) => {
    //console.log(answer);
    state = answer;
    loop();
  });
}

loop();

setInterval(write, 100);

function write() {
  client.write(state);
}