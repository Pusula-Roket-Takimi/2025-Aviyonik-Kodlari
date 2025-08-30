const fs = require("fs");
const os = require("os");
const { SerialPort } = require("serialport");

// ---- CONFIG AYARLARI ----
const portName ="COM5"; //Port Adını yazın
const baudRate = 921600;
const filePath = `${os.homedir()}/Desktop/veri.csv`;
// -----------------

const port = new SerialPort({ path: portName, baudRate: baudRate });

const fileStream = fs.createWriteStream(filePath);

port.on("data", (data) => {
  console.log(data.toString())
  fileStream.write(data);
});

port.on("end", () => {
  fileStream.end();
  console.log("Dosya kaydedildi:", filePath);
});

console.log("Dinleniyor... ESP32 veriyi yollayınca masaüstüne kaydedilecek.");