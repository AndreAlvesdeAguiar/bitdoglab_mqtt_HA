Objetivo:

- O Raspberry Pi Pico W deve se conectar a um servidor MQTT (também conhecido como broker) para enviar e receber mensagens. 

- Conexão ao broker: o cliente se conecta ao servidor MQTT usando o protocolo MQTT.

- Autenticação: o cliente fornecerá credenciais de autenticação, como nome de usuário e senha, para se conectar ao broker.

* Publicação de mensagens:

- Definição do tópico: o cliente define um tópico (ou assunto) para a mensagem que deseja publicar. 

Tópico: /temperature

- Publicação da mensagem: o cliente publica a mensagem no tópico definido.

Neste caso, será publicado o valor processado para o sensor de temperatura do RP2040.

* Inscrição em tópicos:

- Inscrição: o cliente se inscreve em um ou mais tópicos para receber mensagens publicadas nesses tópicos.

Tópico inscrito: /led

- Recebimento de mensagens: o cliente recebe as mensagens publicadas nos tópicos em que se inscreveu.

Neste caso, será obtido o valor do estado do LED presente no microcontrolador – on ou off.
