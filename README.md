# Título do Projeto  

Home Automate


# Objetivo Geral  

Desenvolver um sistema capaz de trazer praticidade e comodidade ao usuário para controlar sua casa de onde estiver através de seu dispositivo móvel ou computador, simulando uma automação residencial, no âmbito das Smart Homes.

Nesse projeto é possível apenas apagar luzes e acionar alarmes, além de monitorar a temperatura de um cômodo.


# Descrição Funcional  

O sistema tem como objetivo monitorar comandos do usuário acerca da casa, além de enviar leituras de sensores para que o usuário possa monitorar o ambiente.

Esse sistema conta com um servidor MQTT instalado e rodando em um dispositivo móvel (para simulação), que pode receber inscrições e publicações de usuários na mesma rede que consigam se autenticar.

É importante salientar que para um projeto real desse porte, o servidor MQTT deve estar disponível em uma máquina dedicada e disponível para a rede externa, sendo possível enviar dados e receber dados mesmo não estando na mesma localidade.

Além do servidor MQTT, o projeto conta com a placa BitDogLab que atua como cliente MQTT, enviando e recebendo dados. A placa simula um alarme, um sensor de temperatura e uma iluminação através de seus periféricos: buzzer, potenciômetro e LED RGB.

A placa que atua como cliente MQTT se inscreve em quatro tópicos:

- **/led**: tópico que faz o controle da iluminação. Ao receber uma publicação neste tópico, a mensagem contida na publicação informa se o LED irá desligar ou ligar.
  - 0 e Off desliga o LED;
  - 1 e On liga o LED;
  - Ao alterar o status do LED, publica no tópico /led/status o seu status atual.
- **/beep**: aciona um alarme de acordo com o número enviado como mensagem da publicação.
- **/ping**: tópico de verificação. Apenas verifica o tempo de atividade da placa.
- **/exit**: ao receber qualquer publicação nesse tópico, a placa encerra suas atividades.

Além dos tópicos que se inscreve, faz publicação em três tópicos diferentes:

- **/temp**: publica informações da temperatura lida a cada 5 segundos.
- **/online**: assim que a placa se conecta ao servidor é enviado uma publicação com a mensagem “1”, para informar que está ativo.
- **/led/status**: Após alterar o status do LED, envia para esse tópico o estado atual do mesmo LED.


# BitDogLab e Código

Quando se trata da BitDogLab, foram utilizados diversos componentes para que o código e o projeto possuam uma interação ímpar com o usuário. Para isso, foi utilizado um display OLED para garantir depurações do que está sendo executado e como está sendo executado. Para depurações mais profundas, foi utilizado a comunicação serial UART, usando o display serial do próprio computador para isso.

Além dos pontos de visualização, foram utilizados o buzzer, para simular o alarme sonoro que pode ser ligado através de um tópico MQTT. Também foi utilizado um LED RGB para simular a iluminação que também pode ser alterada por um tópico MQTT.

Outro ponto referente a componentes é o potenciômetro conectado ao joystick. Esse foi utilizado para simular um sensor de temperatura que tem seus valores publicados em um tópico MQTT (apenas o potenciômetro da direção vertical, o pino 26).

Ademais, foi utilizado, quando se trata de infraestrutura, o módulo WiFi, a fim de conectar a placa à internet e consegue utilizar um servidor MQTT para troca de mensagens.

Foram feitas diversas funções para controle do MQTT, mas as mais importantes se tratam de:
- **start_client()**: função que inicia a conexão;
- **sub_unsub_topics()**: função que executa a inscrição e o cancelamento da mesma nos tópicos;
- **mqtt_incoming_data_cb()**: função que recebe as mensagens dos tópicos inscritos;
- **publish_temperature()**: função que publica a temperatura.

Com isso, foram concluídos os requisitos do projeto, além de conseguir um resultado satisfatório com relação a usabilidade do sistema. O projeto tem opções de depuração e de extensão posterior.

**OBS:** Para utilização, faça um arquivo com nome ``credenciais.h`` e copie o que está em ``credenciais.example.h`` para esse novo arquivo e informe as credenciais de wifi e do servidor MQTT.

# Links para acesso ao vídeo

[Link de vídeo ensaio](https://youtu.be/TBU2JrAHNL0)