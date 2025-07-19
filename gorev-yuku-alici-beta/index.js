const SerialPort = require('serialport');
const port = new SerialPort('/dev/ttyUSB0', { baudRate: 9600 });

let buffer = Buffer.alloc(0);
const PACKET_SIZE = 26; // 1 header + 24 data + 1 footer

port.on('data', (chunk) => {
  buffer = Buffer.concat([buffer, chunk]);

  while (buffer.length >= PACKET_SIZE) {
    const headerIndex = buffer.indexOf(0xAA);
    if (headerIndex === -1) {
      buffer = Buffer.alloc(0); // header yok, buffer temizle
      break;
    }
    if (buffer.length < headerIndex + PACKET_SIZE) {
      // Tam paket gelmedi
      break;
    }

    // Footer kontrolü
    if (buffer[headerIndex + PACKET_SIZE - 1] === 0x55) {
      // Paket tam, veri ayıkla
      const data = buffer.slice(headerIndex + 1, headerIndex + PACKET_SIZE - 1);

      const enlem = data.readFloatLE(0);
      const boylam = data.readFloatLE(4);
      const irtifa = data.readFloatLE(8);
      const basinc = data.readFloatLE(12);
      const yogunluk = data.readFloatLE(16);
      const sicaklik = data.readFloatLE(20);

      console.log({ enlem, boylam, irtifa, basinc, yogunluk, sicaklik });

      // Buffer’dan okunan paketi at
      buffer = buffer.slice(headerIndex + PACKET_SIZE);
    } else {
      // Footer yanlış, header sonrası atla
      buffer = buffer.slice(headerIndex + 1);
    }
  }
});
