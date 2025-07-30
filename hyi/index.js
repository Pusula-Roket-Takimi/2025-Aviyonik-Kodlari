const express = require('express');
const { SerialPort } = require('serialport');
const path = require('path');
const http = require('http');
const WebSocket = require('ws');
const fs = require('fs');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });
const TAKIM_ID = 46;
const PORT = 80;

// Port yönetimi
let aviyonikPort = null;
let gorevPort = null;
let hyiPort = null;
let availablePorts = [];

// Hatalı veri sayaci

let aviyonikErrorCount = 0;
let gorevErrorCount = 0;

const floatFields = [
  'basinc_irtifa', // 4 byte
  'roket_irtifa',  //
  'roket_enlem',  //
  'roket_boylam',
  'gorev_irtifa',
  'gorev_enlem',
  'gorev_boylam',
  'kademe_irtifa',
  'kademe_enlem',
  'kademe_boylam',
  'jiroskop_x',
  'jiroskop_y',
  'jiroskop_z',
  'ivme_x',
  'ivme_y',
  'ivme_z',
  'aci',
];

const HYI_SAF_VERILER = { parasut_durum: 0 };
for (const field of floatFields)
  HYI_SAF_VERILER[field] = 0;



// === PROTOKOL SABİTLERİ ===
const GOREV_HEADER = 0xAA; // Görev yükü header byte
const GOREV_FOOTER = 0x55; // Görev yükü footer byte
const GOREV_FIELDS = [
  { key: 'gorev_enlem', offset: 0 },
  { key: 'gorev_boylam', offset: 4 },
  { key: 'gorev_irtifa', offset: 8 },
  { key: 'basinc', offset: 12 },
  { key: 'yogunluk', offset: 16 },
  { key: 'sicaklik', offset: 20 }
];
const GOREV_PAKET_SIZE = 4 * GOREV_FIELDS.length + 1 + 2; // 6 float + 1 checksum = 27 byte

const AVIYONIK_HEADER = 0xAB; // Aviyonik header byte 
const AVIYONIK_FOOTER = 0x56; // Aviyonik footer byte 
const AVIYONIK_FIELDS = [
  { key: 'roket_enlem', offset: 0 },
  { key: 'roket_boylam', offset: 4 },
  { key: 'roket_irtifa', offset: 8 },
  { key: 'basinc', offset: 12 },
  { key: 'basinc_irtifa', offset: 16 },
  { key: 'ivme_x', offset: 20 },
  { key: 'ivme_y', offset: 24 },
  { key: 'ivme_z', offset: 28 },
  { key: 'jiroskop_x', offset: 32 },
  { key: 'jiroskop_y', offset: 36 },
  { key: 'jiroskop_z', offset: 40 },
  { key: 'aci', offset: 44 },
  { key: 'parasut_durum', offset: 48, type: 'int' } // int32
];
const AVIYONIK_PAKET_SIZE = 4 * 12 + 1 + 1 + 2; // 12 float + 1 int32 + 1 checksum  = 52 byte

// Static dosyaları servis e
app.use(express.static(path.join(__dirname, 'public')));

// Ana sayfa
app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'public', 'index.html'));
});

// Dosyaya yazılan verileri çekmek için endpoint'ler
app.get('/api/gorev-verileri', (req, res) => {
  try {
    if (fs.existsSync('gorev_verileri.txt')) {
      const data = fs.readFileSync('gorev_verileri.txt', 'utf8');
      const lines = data.trim().split('\n');
      const veriler = lines.map(line => {
        const values = line.split(' ');
        return {
          gorev_enlem: parseFloat(values[0]) || 0,
          gorev_boylam: parseFloat(values[1]) || 0,
          gorev_irtifa: parseFloat(values[2]) || 0,
          basinc: parseFloat(values[3]) || 0,
          yogunluk: parseFloat(values[4]) || 0,
          sicaklik: parseFloat(values[5]) || 0,
          timestamp: new Date().toISOString()
        };
      });
      res.json({ success: true, data: veriler });
    } else {
      res.json({ success: true, data: [] });
    }
  } catch (error) {
    console.error('Görev verileri okuma hatası:', error);
    res.status(500).json({ success: false, error: error.message });
  }
});

// Dosyayı temizlemek için endpoint
app.delete('/api/gorev-verileri', (req, res) => {
  try {
    if (fs.existsSync('gorev_verileri.txt')) {
      fs.unlinkSync('gorev_verileri.txt');
      res.json({ success: true, message: 'Dosya silindi' });
    } else {
      res.json({ success: true, message: 'Dosya zaten yok' });
    }
  } catch (error) {
    console.error('Dosya silme hatası:', error);
    res.status(500).json({ success: false, error: error.message });
  }
});

// Dosya boyutunu kontrol etmek için endpoint
app.get('/api/gorev-verileri/size', (req, res) => {
  try {
    if (fs.existsSync('gorev_verileri.txt')) {
      const stats = fs.statSync('gorev_verileri.txt');
      res.json({ 
        success: true, 
        size: stats.size,
        lines: fs.readFileSync('gorev_verileri.txt', 'utf8').split('\n').length - 1
      });
    } else {
      res.json({ success: true, size: 0, lines: 0 });
    }
  } catch (error) {
    console.error('Dosya boyutu kontrol hatası:', error);
    res.status(500).json({ success: false, error: error.message });
  }
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



function sendAviyonikData(ws, msg) {
  if (aviyonikPort?.isOpen)
    aviyonikPort.close();

  aviyonikPort = new SerialPort({
    path: msg.data,
    baudRate: 9600
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
      if (buffer.length < headerIndex + AVIYONIK_PAKET_SIZE)
        break;


      const footerIndex = headerIndex + AVIYONIK_PAKET_SIZE - 1;
      if (buffer[footerIndex] !== AVIYONIK_FOOTER) {
        buffer = buffer.slice(headerIndex + 1);
        continue;
      }

      const payload = buffer.slice(headerIndex, headerIndex + AVIYONIK_PAKET_SIZE); // Tüm paket
      const data = payload.slice(1, AVIYONIK_PAKET_SIZE - 2); // VERILER: header sonrası checksum'a (dahil degil) kadar

      const veri = {};

      for (const field of AVIYONIK_FIELDS)
        veri[field.key] = (field.type === 'int') ? data.readUInt8(field.offset) : data.readFloatLE(field.offset);


      const checksum = payload.readUInt8(AVIYONIK_PAKET_SIZE - 2); // checksum byte !??!?
      const calculated = payload.slice(0, AVIYONIK_PAKET_SIZE - 2).reduce((sum, byte) => sum + byte, 0) % 256;

      buffer = buffer.slice(headerIndex + AVIYONIK_PAKET_SIZE);

      if (checksum !== calculated) {
        console.warn(`⚠️ Checksum hatası: beklenen ${checksum}, hesaplanan ${calculated}`);
        aviyonikErrorCount++;
        ws.send(JSON.stringify({ type: 'aviyonik-error-count-updated', data: aviyonikErrorCount }));
        break;// istemciye gönder bunu
      }
      for (const key in veri)
        HYI_SAF_VERILER[key] = veri[key];

      ws.send(JSON.stringify({ type: 'aviyonik-data', data: veri }));
    }


  });
}


function sendGorevData(ws, msg) {
  if (gorevPort && gorevPort.isOpen) {
    gorevPort.close();
  }
  gorevPort = new SerialPort({
    path: msg.data,
    baudRate: 9600
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
      if (buffer.length < headerIndex + GOREV_PAKET_SIZE) {
        break;
      }
      const footerIndex = headerIndex + GOREV_PAKET_SIZE - 1;
      if (buffer[footerIndex] !== GOREV_FOOTER) {
        buffer = buffer.slice(headerIndex + 1);
        continue;
      }

      const payload = buffer.slice(headerIndex, headerIndex + GOREV_PAKET_SIZE); // Tüm paket
      const data = payload.slice(1, GOREV_PAKET_SIZE - 2); // VERILER: header sonrası checksum'a (dahil degil) kadar

      const veri = {};
      for (const field of GOREV_FIELDS)
        veri[field.key] = data.readFloatLE(field.offset);


      const checksum = payload.readUInt8(GOREV_PAKET_SIZE - 2); // checksum byte
      const calculated = payload.slice(0, GOREV_PAKET_SIZE - 2).reduce((sum, byte) => sum + byte, 0) % 256;

      buffer = buffer.slice(headerIndex + GOREV_PAKET_SIZE);

      if (checksum !== calculated) {
        gorevErrorCount++;
        ws.send(JSON.stringify({ type: 'gorev-error-count-updated', data: gorevErrorCount }));
        break;
      }
      for (const key in veri)
        HYI_SAF_VERILER[key] = veri[key];


      ws.send(JSON.stringify({ type: 'gorev-data', data: veri }));


      const veriArray = GOREV_FIELDS.map(field => veri[field.key]);
      const veriString = veriArray.join(' ') + '\n';
      fs.appendFile('gorev_verileri.txt', veriString, (err) => {
        if (err) {
          console.error('Dosyaya yazma hatası:', err);
        }
      });
    }
  });
}




// HYİ paket oluşturma
function floatToBytesLE(floatVal) {
  let buffer = new ArrayBuffer(4);
  new DataView(buffer).setFloat32(0, floatVal, true);
  return Array.from(new Uint8Array(buffer));
}

let hyiSayac = 0;

function sendHyiData(ws, msg) {
  if (hyiPort?.isOpen) hyiPort.close();

  hyiPort = new SerialPort({
    path: msg.data,
    baudRate: 19200
  });

  hyiPort.on('open', () => {
    console.log('HYİ port açıldı:', msg.data);
    ws.send(JSON.stringify({ type: 'hyi-connected', data: msg.data }));
    setInterval(() => {

      const packet = new Uint8Array(78);
      packet.set([0xFF, 0xFF, 0x54, 0x52], 0);  // HEADER
      packet[4] = TAKIM_ID;
      packet[5] = hyiSayac;

      let offset = 6;
      for (let key of floatFields) {
        const val = parseFloat(HYI_SAF_VERILER[key] || 0);
        packet.set(floatToBytesLE(val), offset);
        offset += 4;
      }

      // Diğer veriler
      packet[74] = HYI_SAF_VERILER.parasut_durum || 0;

      let checksum = 0;
      for (let i = 6; i <= 74; i++) checksum += packet[i];
      packet[75] = checksum % 256;//76
      packet[76] = 0x0D;//77
      packet[77] = 0x0A;//78

      hyiPort.write(packet, (err) => {
        hyiSayac = (++hyiSayac) & 0xFF;
        console.log('HYİ paket gönderildi:', hyiSayac);
        if (err) {
          console.error('HYİ gönderim hatası:', err);
          ws.send(JSON.stringify({ type: 'hyi-send-error', data: err.message }));
        } else {
          console.log('HYİ verisi gönderildi, boyut:', packet.length);
          ws.send(JSON.stringify({ type: 'hyi-sent', data: hyiSayac }));
        }
      });

    }, 200);
  });
}
// --- WebSocket bağlantıları ---
wss.on('connection', (ws) => {
  console.log('Client bağlandı');
  ws.send(JSON.stringify({ type: 'ports-updated', data: availablePorts }));

  ws.on('message', async (message) => {
    let msg;
    try {
      msg = JSON.parse(message);
    } catch (e) {
      console.error('Geçersiz mesaj:', message);
      aviyonikErrorCount++;
      ws.send(JSON.stringify({ type: 'aviyonik-error-count-updated', data: aviyonikErrorCount }));
      return;
    }


    switch (msg.type) {
      case 'refresh-ports':
        await updateAvailablePorts();
        ws.send(JSON.stringify({ type: 'ports-updated', data: availablePorts }));
        break;
      case 'connect-aviyonik':
        try {
          sendAviyonikData(ws, msg);
        } catch (error) {
          console.error('Aviyonik bağlantı hatası:', error);
          ws.send(JSON.stringify({ type: 'aviyonik-error', data: error.message }));
        }
        break;
      case 'connect-gorev':
        try {
          sendGorevData(ws, msg);
        } catch (error) {
          console.error('Görev Yükü bağlantı hatası:', error);
          ws.send(JSON.stringify({ type: 'gorev-error', data: error.message }));
        }
        break;
      case 'connect-hyi':
        try {
          sendHyiData(ws, msg);

        } catch (error) {
          console.error('HYİ bağlantı hatası:', error);
          ws.send(JSON.stringify({ type: 'hyi-error', data: error.message }));
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
