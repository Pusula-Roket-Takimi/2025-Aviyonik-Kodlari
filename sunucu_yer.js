const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const path = require('path');

const app = express();
const server = http.createServer(app);
const io = socketIo(server);

// Static dosyaları servis et
app.use(express.static('public'));

// Ana sayfa
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

class GroundStation {
  constructor() {
    this.aviyonikPort = null;
    this.gorevPort = null;
    this.hyiPort = null;
    
    this.lastAviyonikData = {};
    this.lastGorevData = {};
    this.hyiSayac = 0;
    this.takimId = 42;
    
    this.setupSocketEvents();
  }

  setupSocketEvents() {
    io.on('connection', (socket) => {
      console.log('İstemci bağlandı:', socket.id);
      
      // Port listesi gönder
      this.sendPortList(socket);
      
      // Aviyonik bağlantı
      socket.on('connect-aviyonik', async (portPath) => {
        try {
          await this.connectAviyonik(portPath);
          socket.emit('aviyonik-connected', true);
          this.log('Aviyonik portuna bağlanıldı: ' + portPath);
        } catch (error) {
          socket.emit('aviyonik-connected', false);
          this.log('Aviyonik bağlantı hatası: ' + error.message);
        }
      });
      
      // Görev Yükü bağlantı
      socket.on('connect-gorev', async (portPath) => {
        try {
          await this.connectGorev(portPath);
          socket.emit('gorev-connected', true);
          this.log('Görev Yükü portuna bağlanıldı: ' + portPath);
        } catch (error) {
          socket.emit('gorev-connected', false);
          this.log('Görev Yükü bağlantı hatası: ' + error.message);
        }
      });
      
      // HYİ bağlantı
      socket.on('connect-hyi', async (portPath) => {
        try {
          await this.connectHYI(portPath);
          socket.emit('hyi-connected', true);
          this.log('HYİ portuna bağlanıldı: ' + portPath);
        } catch (error) {
          socket.emit('hyi-connected', false);
          this.log('HYİ bağlantı hatası: ' + error.message);
        }
      });
      
      // Bağlantı koparma
      socket.on('disconnect-aviyonik', () => this.disconnectAviyonik());
      socket.on('disconnect-gorev', () => this.disconnectGorev());
      socket.on('disconnect-hyi', () => this.disconnectHYI());
      
      // HYİ veri gönderimi
      socket.on('send-hyi', () => this.sendHYIData());
      
      // Port listesi yenile
      socket.on('refresh-ports', () => this.sendPortList(socket));
      
      socket.on('disconnect', () => {
        console.log('İstemci bağlantısı kesildi:', socket.id);
      });
    });
  }

  async sendPortList(socket) {
    try {
      const ports = await SerialPort.list();
      const portList = ports.map(port => ({
        path: port.path,
        manufacturer: port.manufacturer,
        serialNumber: port.serialNumber
      }));
      socket.emit('port-list', portList);
    } catch (error) {
      console.error('Port listesi alınamadı:', error);
    }
  }

  async connectAviyonik(portPath) {
    if (this.aviyonikPort && this.aviyonikPort.isOpen) {
      this.aviyonikPort.close();
    }
    
    this.aviyonikPort = new SerialPort({
      path: portPath,
      baudRate: 115200
    });
    
    const parser = this.aviyonikPort.pipe(new ReadlineParser({ delimiter: '\n' }));
    
    parser.on('data', (line) => {
      line = line.trim();
      if (!line) return;
      
      this.log('Aviyonik: ' + line);
      this.parseAviyonikLine(line);
    });
    
    this.aviyonikPort.on('error', (err) => {
      this.log('Aviyonik port hatası: ' + err.message);
      io.emit('aviyonik-connected', false);
    });
  }

  async connectGorev(portPath) {
    if (this.gorevPort && this.gorevPort.isOpen) {
      this.gorevPort.close();
    }
    
    this.gorevPort = new SerialPort({
      path: portPath,
      baudRate: 115200
    });
    
    const parser = this.gorevPort.pipe(new ReadlineParser({ delimiter: '\n' }));
    
    parser.on('data', (line) => {
      line = line.trim();
      if (!line) return;
      
      this.log('Görev Yükü: ' + line);
      this.parseGorevLine(line);
    });
    
    this.gorevPort.on('error', (err) => {
      this.log('Görev Yükü port hatası: ' + err.message);
      io.emit('gorev-connected', false);
    });
  }

  async connectHYI(portPath) {
    if (this.hyiPort && this.hyiPort.isOpen) {
      this.hyiPort.close();
    }
    
    this.hyiPort = new SerialPort({
      path: portPath,
      baudRate: 19200
    });
    
    this.hyiPort.on('error', (err) => {
      this.log('HYİ port hatası: ' + err.message);
      io.emit('hyi-connected', false);
    });
  }

  disconnectAviyonik() {
    if (this.aviyonikPort && this.aviyonikPort.isOpen) {
      this.aviyonikPort.close();
      this.log('Aviyonik port kapatıldı');
      io.emit('aviyonik-connected', false);
    }
  }

  disconnectGorev() {
    if (this.gorevPort && this.gorevPort.isOpen) {
      this.gorevPort.close();
      this.log('Görev Yükü port kapatıldı');
      io.emit('gorev-connected', false);
    }
  }

  disconnectHYI() {
    if (this.hyiPort && this.hyiPort.isOpen) {
      this.hyiPort.close();
      this.log('HYİ port kapatıldı');
      io.emit('hyi-connected', false);
    }
  }

  parseAviyonikLine(line) {
    if (line.startsWith('#BOD,') && line.endsWith('#EOD')) {
      const content = line.slice(5, -4);
      const parsed = this.parseKeyValueString(content);
      this.lastAviyonikData = parsed;
      io.emit('aviyonik-data', parsed);
    } else {
      this.log('Bilinmeyen aviyonik format: ' + line);
    }
  }

  parseGorevLine(line) {
    if (line.startsWith('#BOD_Gorev,') && line.endsWith('#EOD_Gorev')) {
      const content = line.slice(11, -10);
      const parsed = this.parseKeyValueString(content);
      this.lastGorevData = parsed;
      io.emit('gorev-data', parsed);
    } else if (line.startsWith('#BOD,') && line.endsWith('#EOD')) {
      const content = line.slice(5, -4);
      const parsed = this.parseKeyValueString(content);
      this.lastGorevData = parsed;
      io.emit('gorev-data', parsed);
    } else {
      this.log('Bilinmeyen görev yükü format: ' + line);
    }
  }

  parseKeyValueString(str) {
    const obj = {};
    const parts = str.split(',');
    for (let part of parts) {
      const [key, val] = part.split('=');
      if (key && val !== undefined) {
        obj[key.trim()] = val.trim();
      }
    }
    return obj;
  }

  floatToBytesLE(floatVal) {
    const buffer = Buffer.allocUnsafe(4);
    buffer.writeFloatLE(floatVal, 0);
    return Array.from(buffer);
  }

  buildHYIPacket() {
    const packet = Buffer.alloc(78);
    packet.writeUInt8(0xFF, 0);
    packet.writeUInt8(0xFF, 1);
    packet.writeUInt8(0x54, 2);
    packet.writeUInt8(0x52, 3);
    packet.writeUInt8(this.takimId, 4);
    packet.writeUInt8(this.hyiSayac, 5);

    const safeFloat = (obj, key) => parseFloat(obj[key] || 0);

    // Aviyonik verilerinden
    const irtifa = safeFloat(this.lastAviyonikData, 'PI');
    const gpsIrtifa = safeFloat(this.lastAviyonikData, 'GI');
    const enlem = safeFloat(this.lastAviyonikData, 'E');
    const boylam = safeFloat(this.lastAviyonikData, 'B');
    const gyroX = safeFloat(this.lastAviyonikData, 'GX') * Math.PI / 180;
    const gyroY = safeFloat(this.lastAviyonikData, 'GY') * Math.PI / 180;
    const gyroZ = safeFloat(this.lastAviyonikData, 'GZ') * Math.PI / 180;
    const accX = safeFloat(this.lastAviyonikData, 'AX') * 9.80665;
    const accY = safeFloat(this.lastAviyonikData, 'AY') * 9.80665;
    const accZ = safeFloat(this.lastAviyonikData, 'AZ') * 9.80665;
    const angle = safeFloat(this.lastAviyonikData, 'RGZ');
    const durum = parseInt(this.lastAviyonikData['PD'] || 0);

    // Görev yükü verilerinden
    const gGI = safeFloat(this.lastGorevData, 'GI');
    const gE = safeFloat(this.lastGorevData, 'E');
    const gB = safeFloat(this.lastGorevData, 'B');

    packet.writeFloatLE(irtifa, 6);
    packet.writeFloatLE(gpsIrtifa, 10);
    packet.writeFloatLE(enlem, 14);
    packet.writeFloatLE(boylam, 18);
    packet.writeFloatLE(gGI, 22);
    packet.writeFloatLE(gE, 26);
    packet.writeFloatLE(gB, 30);
    packet.writeFloatLE(gyroX, 46);
    packet.writeFloatLE(gyroY, 50);
    packet.writeFloatLE(gyroZ, 54);
    packet.writeFloatLE(accX, 58);
    packet.writeFloatLE(accY, 62);
    packet.writeFloatLE(accZ, 66);
    packet.writeFloatLE(angle, 70);
    packet.writeUInt8(durum, 74);

    let checksum = 0;
    for (let i = 4; i <= 74; i++) {
      checksum += packet[i];
    }
    packet.writeUInt8(checksum % 256, 75);
    packet.writeUInt8(0x0D, 76);
    packet.writeUInt8(0x0A, 77);

    return packet;
  }

  async sendHYIData() {
    if (!this.hyiPort || !this.hyiPort.isOpen) {
      this.log('HYİ portu bağlı değil!');
      return;
    }

    try {
      const packet = this.buildHYIPacket();
      this.hyiSayac = (this.hyiSayac + 1) % 256;
      
      await new Promise((resolve, reject) => {
        this.hyiPort.write(packet, (err) => {
          if (err) reject(err);
          else resolve();
        });
      });
      
      this.log(`HYİ paketi gönderildi! (Sayaç: ${this.hyiSayac - 1})`);
      io.emit('hyi-sent', true);
    } catch (error) {
      this.log('HYİ gönderim hatası: ' + error.message);
      io.emit('hyi-sent', false);
    }
  }

  log(message) {
    const timestamp = new Date().toLocaleTimeString();
    const logMessage = `[${timestamp}] ${message}`;
    console.log(logMessage);
    io.emit('log', logMessage);
  }
}

// Yer istasyonu başlat
const groundStation = new GroundStation();

const PORT = process.env.PORT || 3000;
server.listen(PORT, () => {
  console.log(`Yer İstasyonu sunucusu http://localhost:${PORT} adresinde çalışıyor`);
});
