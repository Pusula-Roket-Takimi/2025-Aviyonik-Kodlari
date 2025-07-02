const express = require('express');
const { SerialPort } = require('serialport');
const { ReadlineParser } = require('@serialport/parser-readline');
const path = require('path');
const http = require('http');
const socketIo = require('socket.io');

const app = express();
const server = http.createServer(app);
const io = socketIo(server, {
  cors: {
    origin: "*",
    methods: ["GET", "POST"]
  }
});

const PORT = 3000;

// Port yönetimi
let aviyonikPort = null;
let gorevPort = null;
let hyiPort = null;
let availablePorts = [];

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
    io.emit('ports-updated', availablePorts);
  } catch (error) {
    console.error('Port listeleme hatası:', error);
  }
}

// Socket bağlantıları
io.on('connection', (socket) => {
  console.log('Client bağlandı:', socket.id);
  
  // Client'a mevcut portları gönder
  socket.emit('ports-updated', availablePorts);
  
  // Port listesi güncelleme isteği
  socket.on('refresh-ports', async () => {
    await updateAvailablePorts();
  });

  // Aviyonik port bağlantısı
  socket.on('connect-aviyonik', async (portPath) => {
    try {
      if (aviyonikPort && aviyonikPort.isOpen) {
        aviyonikPort.close();
      }
      
      aviyonikPort = new SerialPort({
        path: portPath,
        baudRate: 115200
      });

      const parser = aviyonikPort.pipe(new ReadlineParser({ delimiter: '\n' }));
      
      aviyonikPort.on('open', () => {
        console.log('Aviyonik port açıldı:', portPath);
        socket.emit('aviyonik-connected', portPath);
      });

      aviyonikPort.on('error', (err) => {
        console.error('Aviyonik port hatası:', err);
        socket.emit('aviyonik-error', err.message);
      });

      parser.on('data', (data) => {
        const cleanData = data.toString().trim();
        if (cleanData) {
          socket.emit('aviyonik-data', cleanData);
        }
      });

    } catch (error) {
      console.error('Aviyonik bağlantı hatası:', error);
      socket.emit('aviyonik-error', error.message);
    }
  });

  // Görev Yükü port bağlantısı
  socket.on('connect-gorev', async (portPath) => {
    try {
      if (gorevPort && gorevPort.isOpen) {
        gorevPort.close();
      }
      
      gorevPort = new SerialPort({
        path: portPath,
        baudRate: 115200
      });

      const parser = gorevPort.pipe(new ReadlineParser({ delimiter: '\n' }));
      
      gorevPort.on('open', () => {
        console.log('Görev Yükü port açıldı:', portPath);
        socket.emit('gorev-connected', portPath);
      });

      gorevPort.on('error', (err) => {
        console.error('Görev Yükü port hatası:', err);
        socket.emit('gorev-error', err.message);
      });

      parser.on('data', (data) => {
        const cleanData = data.toString().trim();
        if (cleanData) {
          socket.emit('gorev-data', cleanData);
        }
      });

    } catch (error) {
      console.error('Görev Yükü bağlantı hatası:', error);
      socket.emit('gorev-error', error.message);
    }
  });

  // HYİ port bağlantısı
  socket.on('connect-hyi', async (portPath) => {
    try {
      if (hyiPort && hyiPort.isOpen) {
        hyiPort.close();
      }
      
      hyiPort = new SerialPort({
        path: portPath,
        baudRate: 19200
      });
      
      hyiPort.on('open', () => {
        console.log('HYİ port açıldı:', portPath);
        socket.emit('hyi-connected', portPath);
      });

      hyiPort.on('error', (err) => {
        console.error('HYİ port hatası:', err);
        socket.emit('hyi-error', err.message);
      });

    } catch (error) {
      console.error('HYİ bağlantı hatası:', error);
      socket.emit('hyi-error', error.message);
    }
  });

  // HYİ veri gönderimi
  socket.on('send-hyi-data', (data) => {
    try {
      if (hyiPort && hyiPort.isOpen) {
        const buffer = Buffer.from(data);
        hyiPort.write(buffer, (err) => {
          if (err) {
            console.error('HYİ gönderim hatası:', err);
            socket.emit('hyi-send-error', err.message);
          } else {
            console.log('HYİ verisi gönderildi, boyut:', buffer.length);
            socket.emit('hyi-sent', buffer.length);
          }
        });
      } else {
        socket.emit('hyi-send-error', 'HYİ portu açık değil');
      }
    } catch (error) {
      console.error('HYİ gönderim hatası:', error);
      socket.emit('hyi-send-error', error.message);
    }
  });

  // Port kapatma işlemleri
  socket.on('disconnect-aviyonik', () => {
    if (aviyonikPort && aviyonikPort.isOpen) {
      aviyonikPort.close(() => {
        console.log('Aviyonik port kapatıldı');
        socket.emit('aviyonik-disconnected');
      });
    }
  });

  socket.on('disconnect-gorev', () => {
    if (gorevPort && gorevPort.isOpen) {
      gorevPort.close(() => {
        console.log('Görev Yükü port kapatıldı');
        socket.emit('gorev-disconnected');
      });
    }
  });

  socket.on('disconnect-hyi', () => {
    if (hyiPort && hyiPort.isOpen) {
      hyiPort.close(() => {
        console.log('HYİ port kapatıldı');
        socket.emit('hyi-disconnected');
      });
    }
  });

  // Client bağlantısı koptuğunda
  socket.on('disconnect', () => {
    console.log('Client bağlantısı koptu:', socket.id);
  });
});

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
