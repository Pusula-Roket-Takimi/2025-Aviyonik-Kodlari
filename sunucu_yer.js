const express = require('express');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const cors = require('cors');
const path = require('path');

const app = express();
app.use(cors());
app.use(express.json());
app.use(express.static(path.join(__dirname, 'public')));

// Veri depolama
let lastAviyonikData = {};
let lastGorevData = {};
let hyiSayac = 0;
const takimId = 42;

// Serial port bağlantıları
let dataPort = null;
let dataParser = null;
let hyiPort = null;

// API Endpoint'leri

// Port listesini getir
app.get('/api/ports', async (req, res) => {
  try {
    const ports = await SerialPort.list();
    res.json(ports);
  } catch (err) {
    res.status(500).json({ error: 'Port listesi alınamadı', details: err.message });
  }
});

// Veri portuna bağlan
app.post('/api/connect', async (req, res) => {
  try {
    const { portPath } = req.body;
    
    if (dataPort && dataPort.isOpen) {
      await new Promise((resolve) => dataPort.close(resolve));
    }
    
    dataPort = new SerialPort({ path: portPath, baudRate: 115200 });
    
    const parser = dataPort.pipe(new ReadlineParser({ delimiter: '\n' }));
    parser.on('data', parseLine);
    
    dataPort.on('error', (err) => {
      console.error('Data port error:', err);
    });
    
    res.json({ success: true, message: `Port ${portPath} bağlantısı başarılı` });
  } catch (err) {
    res.status(500).json({ success: false, message: err.message });
  }
});

// Veri portundan bağlantıyı kes
app.post('/api/disconnect', async (req, res) => {
  try {
    if (dataPort) {
      await new Promise((resolve) => dataPort.close(resolve));
      dataPort = null;
    }
    res.json({ success: true, message: 'Port bağlantısı kesildi' });
  } catch (err) {
    res.status(500).json({ success: false, message: err.message });
  }
});

// HYI portuna bağlan
app.post('/api/hyi/connect', async (req, res) => {
  try {
    const { portPath } = req.body;
    
    if (hyiPort && hyiPort.isOpen) {
      await new Promise((resolve) => hyiPort.close(resolve));
    }
    
    hyiPort = new SerialPort({ path: portPath, baudRate: 19200 });
    
    hyiPort.on('error', (err) => {
      console.error('HYI port error:', err);
    });
    
    res.json({ success: true, message: `HYI port ${portPath} bağlantısı başarılı` });
  } catch (err) {
    res.status(500).json({ success: false, message: err.message });
  }
});

// HYI verisi gönder
app.post('/api/hyi/send', async (req, res) => {
  try {
    if (!hyiPort || !hyiPort.isOpen) {
      throw new Error('HYI port bağlı değil');
    }
    
    const packet = buildHYIPacket();
    hyiSayac = (hyiSayac + 1) % 256;
    
    hyiPort.write(packet, (err) => {
      if (err) throw err;
      res.json({ success: true, message: 'HYI paketi gönderildi' });
    });
  } catch (err) {
    res.status(500).json({ success: false, message: err.message });
  }
});

// Son verileri getir
app.get('/api/aviyonik', (req, res) => {
  res.json(lastAviyonikData);
});

app.get('/api/gorev', (req, res) => {
  res.json(lastGorevData);
});

// Yardımcı Fonksiyonlar

function parseLine(line) {
  line = line.trim();
  console.log('Gelen veri:', line);
  
  if (line.startsWith('#BOD,') && line.endsWith('#EOD')) {
    const content = line.slice(5, -4);
    lastAviyonikData = parseKeyValueString(content);
  } else if (line.startsWith('#BOD_Gorev,') && line.endsWith('#EOD_Gorev')) {
    const content = line.slice(10, -9);
    lastGorevData = parseKeyValueString(content);
  }
}

function parseKeyValueString(str) {
  const obj = {};
  const parts = str.split(',');
  parts.forEach(part => {
    const [key, val] = part.split('=');
    if (key && val !== undefined) obj[key.trim()] = val.trim();
  });
  return obj;
}

function floatToBytesLE(floatVal) {
  const buffer = new ArrayBuffer(4);
  new DataView(buffer).setFloat32(0, floatVal, true);
  return new Uint8Array(buffer);
}

function buildHYIPacket() {
  const packet = new Uint8Array(78);
  packet.set([0xFF, 0xFF, 0x54, 0x52], 0);
  packet[4] = takimId;
  packet[5] = hyiSayac;

  function safeFloat(obj, key) {
    return parseFloat(obj[key] || 0);
  }

  const irtifa = safeFloat(lastAviyonikData, 'PI');
  const gpsIrtifa = safeFloat(lastAviyonikData, 'GI');
  const enlem = safeFloat(lastAviyonikData, 'E');
  const boylam = safeFloat(lastAviyonikData, 'B');
  const gGI = safeFloat(lastGorevData, 'GI');
  const gE = safeFloat(lastGorevData, 'E');
  const gB = safeFloat(lastGorevData, 'B');
  const gyroX = safeFloat(lastAviyonikData, 'GX') * Math.PI / 180;
  const gyroY = safeFloat(lastAviyonikData, 'GY') * Math.PI / 180;
  const gyroZ = safeFloat(lastAviyonikData, 'GZ') * Math.PI / 180;
  const accX = safeFloat(lastAviyonikData, 'AX') * 9.80665;
  const accY = safeFloat(lastAviyonikData, 'AY') * 9.80665;
  const accZ = safeFloat(lastAviyonikData, 'AZ') * 9.80665;
  const angle = safeFloat(lastAviyonikData, 'RGZ');
  const durum = parseInt(lastAviyonikData['PD'] || 0);

  packet.set(floatToBytesLE(irtifa), 6);
  packet.set(floatToBytesLE(gpsIrtifa), 10);
  packet.set(floatToBytesLE(enlem), 14);
  packet.set(floatToBytesLE(boylam), 18);
  packet.set(floatToBytesLE(gGI), 22);
  packet.set(floatToBytesLE(gE), 26);
  packet.set(floatToBytesLE(gB), 30);
  packet.set(floatToBytesLE(gyroX), 46);
  packet.set(floatToBytesLE(gyroY), 50);
  packet.set(floatToBytesLE(gyroZ), 54);
  packet.set(floatToBytesLE(accX), 58);
  packet.set(floatToBytesLE(accY), 62);
  packet.set(floatToBytesLE(accZ), 66);
  packet.set(floatToBytesLE(angle), 70);
  packet[74] = durum;

  let checksum = 0;
  for (let i = 4; i <= 74; i++) checksum += packet[i];
  packet[75] = checksum % 256;
  packet[76] = 0x0D;
  packet[77] = 0x0A;

  return packet;
}

// 404 Handler
app.use((req, res) => {
  res.status(404).json({ error: 'Endpoint bulunamadı' });
});

// Sunucuyu başlat
const PORT = process.env.PORT || 3000;
app.listen(PORT, () => {
  console.log(`Sunucu http://localhost:${PORT} adresinde çalışıyor`);
});