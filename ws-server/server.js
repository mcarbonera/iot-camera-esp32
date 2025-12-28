import http from "http";
import { WebSocketServer } from "ws";
import fs from "fs";
import path from "path";
import { fileURLToPath } from "url";

const __filename = fileURLToPath(import.meta.url);
const __dirname = path.dirname(__filename);

// Servidor HTTP para HTML estático
const server = http.createServer((req, res) => {
  const file = req.url === "/" ? "index.html" : req.url;
  const filePath = path.join(__dirname, "public", file);

  fs.readFile(filePath, (err, data) => {
    if (err) {
      res.writeHead(404);
      res.end("Not found");
      return;
    }

    res.writeHead(200);
    res.end(data);
  });
});

// WebSocket
const wss = new WebSocketServer({ server });
const clients = new Set();

wss.on("connection", (ws) => {
  clients.add(ws);
  console.log("WebSocket conectado");

  ws.on("message", (data, isBinary) => {
    for (const client of clients) {
      if (client === ws) continue; // ignora quem enviou
      if (client.readyState !== client.OPEN) continue;

      if (isBinary) {
        client.send(data, { binary: true });
      } else {
        client.send(data.toString(), { binary: false });
      }
    }
  });

  ws.on("close", () => {
    clients.delete(ws);
    console.log("WebSocket desconectado");
  });

  ws.on("error", (err) => {
    console.error("Erro no WebSocket:", err.message);
    clients.delete(ws);
  });
});

server.listen(3000, () => {
  console.log("Servidor rodando em http://localhost:3000");
});