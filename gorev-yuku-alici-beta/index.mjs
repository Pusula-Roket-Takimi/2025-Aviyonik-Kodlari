import { SerialPort } from 'serialport';

const kartlar = await SerialPort.list();
console.log(kartlar);

const path = kartlar.find(port => port.manufacturer === "Prolific")?.path || 'COM6';

const port = new SerialPort({ path, baudRate: 9600 });

let buffer = Buffer.alloc(0);
const PACKET_SIZE = 27; // 1 header + 6 * 4 byte float + 1 checksum + 1 footer

port.on('data', (chunk) => {
  console.log('BUFFER:', chunk);
  buffer = Buffer.concat([buffer, chunk]);

  while (buffer.length >= PACKET_SIZE) {
    const headerIndex = buffer.indexOf(0xAA); // HEADER BYTE
    if (headerIndex === -1) {
      buffer = Buffer.alloc(0);
      break;
    }

    if (buffer.length < headerIndex + PACKET_SIZE) {
      break;
    }

    const footerIndex = headerIndex + PACKET_SIZE - 1;

    if (buffer[footerIndex] === 0x55) { // FOOTER BYTE
      const payload = buffer.slice(headerIndex, headerIndex + PACKET_SIZE); // Tüm paket
      const data = payload.slice(1, PACKET_SIZE - 2); // header sonrası checksum'a kadar

      const enlem = data.readFloatLE(0);
      const boylam = data.readFloatLE(4);
      const irtifa = data.readFloatLE(8);
      const basinc = data.readFloatLE(12);
      const yogunluk = data.readFloatLE(16);
      const sicaklik = data.readFloatLE(20);

      const checksum = payload.readUInt8(25); // checksum byte
      const calculated = payload.slice(0, PACKET_SIZE - 2).reduce((sum, byte) => sum + byte, 0) % 256;



      if (checksum === calculated) {
        console.log('✅ Geçerli veri:', { enlem, boylam, irtifa, basinc, yogunluk, sicaklik, calculated });
      } else {
        console.warn(`⚠️ Checksum hatası: beklenen ${checksum}, hesaplanan ${calculated}`);
      }

      // İşlenmiş veriyi buffer’dan sil
      buffer = buffer.slice(headerIndex + PACKET_SIZE);
    } else {
      // Footer hatalıysa, header’dan sonrasını kırp
      buffer = buffer.slice(headerIndex + 1);
    }
  }
});
