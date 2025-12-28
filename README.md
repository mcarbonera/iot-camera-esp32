# ESP32 + Camera OV7670

Trabalho final da disciplina de Introdução ao Desenvolvimento para IOT da Especialização em Programação para Dispositivos Móveis - UTFPR.

- Usando ESP-IDF.
- Servidor websocket em nodeJS.
- Página html para ver as imagens.
- Comandos enviados ao ESP32 para modificar a imagem para ASCII.

# Para rodar:

- Na pasta ws-server, executar com:

```
npm start
```

- Na pasta esp-camera, utilizar a extensão platformio e fazer o upload do código para o ESP32.
- A pinagem para conexão entre câmera e o microcontrolador pode ser vista na variável "config", no arquivo main.c.