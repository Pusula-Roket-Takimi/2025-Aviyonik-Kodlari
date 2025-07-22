const express = require('express');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const path = require('path');
const http = require('http');
const WebSocket = require('ws');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

const PORT = 3000;

// Port yönetimi
let aviyonikPort = null;
let gorevPort = null;
let hyiPort = null;
let availablePorts = [];

// === PROTOKOL SABİTLERİ ===
const GOREV_HEADER = 0xAA; // Görev yükü header byte
const GOREV_FOOTER = 0x55; // Görev yükü footer byte
const GOREV_FIELDS = [
  { key: 'enlem', offset: 0 },
  { key: 'boylam', offset: 4 },
  { key: 'irtifa', offset: 8 },
  { key: 'basinc', offset: 12 },
  { key: 'yogunluk', offset: 16 },
  { key: 'sicaklik', offset: 20 }
];
const GOREV_PAKET_SIZE = 4 * GOREV_FIELDS.length; // 6 float = 24 byte

const AVIYONIK_HEADER = 0xAB; // Aviyonik header byte (örnek)
const AVIYONIK_FOOTER = 0x56; // Aviyonik footer byte (örnek)
const AVIYONIK_FIELDS = [
  { key: 'enlem', offset: 0 },
  { key: 'boylam', offset: 4 },
  { key: 'gps_irtifa', offset: 8 },
  { key: 'basinc', offset: 12 },
  { key: 'basinc_irtifa', offset: 16 },
  { key: 'ivmex', offset: 20 },
  { key: 'ivmey', offset: 24 },
  { key: 'ivmez', offset: 28 },
  { key: 'gyrox', offset: 32 },
  { key: 'gyroy', offset: 36 },
  { key: 'gyroz', offset: 40 },
  { key: 'parasut_durum', offset: 44, type: 'int' } // int32
];
const AVIYONIK_PAKET_SIZE = 4 * 11 + 4; // 11 float + 1 int32 = 48 byte

// Static dosyaları servis et
app.use(express.static(path.join(__dirname, 'public')));

// Ana sayfa
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Mevcut portları listele
async function updateAvailablePorts() {
  try {
    const ports = await SerialPort.list();
    availablePorts = ports.map(port => ({
      path: port.path,
      manufacturer: port.manufacturer || 'Bilinmiyor',
      serialNumber: port.serialNumber || 'Yok'
    }));
    broadcast('ports-updated', availablePorts);
  } catch (error) {
    console.error('Port listeleme hatası:', error);
  }
}

// --- WebSocket bağlantıları ---
wss.on('connection', (ws) => {
  console.log('Client bağlandı');

  // Client'a mevcut portları gönder
  ws.send(JSON.stringify({ type: 'ports-updated', data: availablePorts }));

  ws.on('message', async (message) => {
    let msg;
    try {
      msg = JSON.parse(message);
    } catch (e) {
      console.error('Geçersiz mesaj:', message);
      return;
    }
    // Mesaj tipine göre işle
    switch (msg.type) {
      case 'refresh-ports':
        await updateAvailablePorts();
        ws.send(JSON.stringify({ type: 'ports-updated', data: availablePorts }));
        break;
      case 'connect-aviyonik':
        try {
          if (aviyonikPort && aviyonikPort.isOpen) {
            aviyonikPort.close();
          }
          aviyonikPort = new SerialPort({
            path: msg.data,
            baudRate: 115200
          });
          let buffer = Buffer.alloc(0);
          aviyonikPort.on('open', () => {
            console.log('Aviyonik port açıldı:', msg.data);
            ws.send(JSON.stringify({ type: 'aviyonik-connected', data: msg.data }));
          });
          aviyonikPort.on('error', (err) => {
            console.error('Aviyonik port hatası:', err);
            ws.send(JSON.stringify({ type: 'aviyonik-error', data: err.message }));
          });
          aviyonikPort.on('data', (data) => {
            buffer = Buffer.concat([buffer, data]);
            while (buffer.length > 0) {
              const headerIndex = buffer.indexOf(AVIYONIK_HEADER);
              if (headerIndex === -1) {
                buffer = Buffer.alloc(0);
                break;
              }
              if (buffer.length < headerIndex + 1 + AVIYONIK_PAKET_SIZE + 1) {
                break;
              }
              const paketStart = headerIndex + 1;
              const paketEnd = paketStart + AVIYONIK_PAKET_SIZE;
              const footerIndex = paketEnd;
              if (buffer[footerIndex] !== AVIYONIK_FOOTER) {
                buffer = buffer.slice(headerIndex + 1);
                continue;
              }
              const paketBuffer = buffer.slice(paketStart, paketEnd);
              const veri = {};
              for (const field of AVIYONIK_FIELDS) {
                if (field.type === 'int') {
                  veri[field.key] = paketBuffer.readInt32LE(field.offset);
                } else {
                  veri[field.key] = paketBuffer.readFloatLE(field.offset);
                }
              }
              ws.send(JSON.stringify({ type: 'aviyonik-data', data: veri }));
              buffer = buffer.slice(footerIndex + 1);
            }
          });
        } catch (error) {
          console.error('Aviyonik bağlantı hatası:', error);
          ws.send(JSON.stringify({ type: 'aviyonik-error', data: error.message }));
        }
        break;
      case 'connect-gorev':
        try {
          if (gorevPort && gorevPort.isOpen) {
            gorevPort.close();
          }
          gorevPort = new SerialPort({
            path: msg.data,
            baudRate: 115200
          });
          let buffer = Buffer.alloc(0);
          gorevPort.on('open', () => {
            console.log('Görev Yükü port açıldı:', msg.data);
            ws.send(JSON.stringify({ type: 'gorev-connected', data: msg.data }));
          });
          gorevPort.on('error', (err) => {
            console.error('Görev Yükü port hatası:', err);
            ws.send(JSON.stringify({ type: 'gorev-error', data: err.message }));
          });
          gorevPort.on('data', (data) => {
            buffer = Buffer.concat([buffer, data]);
            while (buffer.length > 0) {
              const headerIndex = buffer.indexOf(GOREV_HEADER);
              if (headerIndex === -1) {
                buffer = Buffer.alloc(0);
                break;
              }
              if (buffer.length < headerIndex + 1 + GOREV_PAKET_SIZE + 1) {
                break;
              }
              const paketStart = headerIndex + 1;
              const paketEnd = paketStart + GOREV_PAKET_SIZE;
              const footerIndex = paketEnd;
              if (buffer[footerIndex] !== GOREV_FOOTER) {
                buffer = buffer.slice(headerIndex + 1);
                continue;
              }
              const paketBuffer = buffer.slice(paketStart, paketEnd);
              const veri = {};
              for (const field of GOREV_FIELDS) {
                veri[field.key] = paketBuffer.readFloatLE(field.offset);
              }
              ws.send(JSON.stringify({ type: 'gorev-data', data: veri }));
              buffer = buffer.slice(footerIndex + 1);
            }
          });
        } catch (error) {
          console.error('Görev Yükü bağlantı hatası:', error);
          ws.send(JSON.stringify({ type: 'gorev-error', data: error.message }));
        }
        break;
      case 'connect-hyi':
        try {
          if (hyiPort && hyiPort.isOpen) {
            hyiPort.close();
          }
          
          hyiPort = new SerialPort({
            path: msg.data,
            baudRate: 19200
          });
          
          hyiPort.on('open', () => {
            console.log('HYİ port açıldı:', msg.data);
            ws.send(JSON.stringify({ type: 'hyi-connected', data: msg.data }));
          });

          hyiPort.on('error', (err) => {
            console.error('HYİ port hatası:', err);
            ws.send(JSON.stringify({ type: 'hyi-error', data: err.message }));
          });

        } catch (error) {
          console.error('HYİ bağlantı hatası:', error);
          ws.send(JSON.stringify({ type: 'hyi-error', data: error.message }));
        }
        break;
      case 'send-hyi-data':
        try {
          if (hyiPort && hyiPort.isOpen) {
            const buffer = Buffer.from(msg.data);
            hyiPort.write(buffer, (err) => {
              if (err) {
                console.error('HYİ gönderim hatası:', err);
                ws.send(JSON.stringify({ type: 'hyi-send-error', data: err.message }));
              } else {
                console.log('HYİ verisi gönderildi, boyut:', buffer.length);
                ws.send(JSON.stringify({ type: 'hyi-sent', data: buffer.length }));
              }
            });
          } else {
            ws.send(JSON.stringify({ type: 'hyi-send-error', data: 'HYİ portu açık değil' }));
          }
        } catch (error) {
          console.error('HYİ gönderim hatası:', error);
          ws.send(JSON.stringify({ type: 'hyi-send-error', data: error.message }));
        }
        break;
      case 'disconnect-aviyonik':
        if (aviyonikPort && aviyonikPort.isOpen) {
          aviyonikPort.close(() => {
            console.log('Aviyonik port kapatıldı');
            ws.send(JSON.stringify({ type: 'aviyonik-disconnected' }));
          });
        }
        break;
      case 'disconnect-gorev':
        if (gorevPort && gorevPort.isOpen) {
          gorevPort.close(() => {
            console.log('Görev Yükü port kapatıldı');
            ws.send(JSON.stringify({ type: 'gorev-disconnected' }));
          });
        }
        break;
      case 'disconnect-hyi':
        if (hyiPort && hyiPort.isOpen) {
          hyiPort.close(() => {
            console.log('HYİ port kapatıldı');
            ws.send(JSON.stringify({ type: 'hyi-disconnected' }));
          });
        }
        break;
      default:
        break;
    }
  });

  ws.on('close', () => {
    console.log('Client bağlantısı koptu');
  });
});

// Port eventlerinde io.emit yerine, bağlı tüm ws client'lara gönder
function broadcast(type, data) {
  wss.clients.forEach(client => {
    if (client.readyState === WebSocket.OPEN) {
      client.send(JSON.stringify({ type, data }));
    }
  });
}

// Sunucuyu başlat
server.listen(PORT, async () => {
  console.log(`Sunucu ${PORT} portunda çalışıyor`);
  console.log(`http://localhost:${PORT} adresini ziyaret edin`);
  
  // İlk port listesini güncelle
  await updateAvailablePorts();
  
  // Port listesini periyodik olarak güncelle
  setInterval(updateAvailablePorts, 5000);
});

// Uygulama kapanırken portları kapat
process.on('SIGINT', () => {
  console.log('Uygulama kapatılıyor...');
  
  if (aviyonikPort && aviyonikPort.isOpen) {
    aviyonikPort.close();
  }
  if (gorevPort && gorevPort.isOpen) {
    gorevPort.close();
  }
  if (hyiPort && hyiPort.isOpen) {
    hyiPort.close();
  }
  
  process.exit(0);
});
