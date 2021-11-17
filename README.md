# Cinedrive
Interactive movie controller
Archivos de código de Arduino y XOD
Objetivos del proyecto: 
1-Unificar códigos de modulos del prototipo en un mismo archivo.
2-Traducir código de XOD a Arduino IDE (Modulo de sonido CATALEX MP3 SERIAL PLAYER Y NEOPIXEL RING 16 LEDS)
3-CONECTAR CONTROL CON UNITY POR MEDIO DE COMUNICACIÓN SERIAL.

INDICE DE COMPONENTES CONTROL:
-LUCES RGB NEOPIXEL
-BOTONES E INTERRUPTORES
-ROTARY ENCODER
-MP3 SERIAL PLAYER YX5300
-RFID-PN-532 (READ/WRITE TAG)
-JOYSTICK

Botones, interruptores, llave, perilla, joystick y Tarjetas accionan:
-Aro de luz con animación en sus 16 leds (dependiendo de la acción ejecutada).
-Archivo de audio desde el MODULO MP3 SERIAL PLAYER YX5300. Los archivos están almacenados en la tarjeta micro-sd y tienen por nombre 000, 001, 002....
-Perilla mueve los leds en secuencia replicando el movimiento de giro.

COMUNICACIÓN SERIAL CON UNITY:
El Dispositivo se conecta por medio de bluetooth gracias al microcontrolador ESP-32. 
Unity debe reconocer los botones accionados, perilla, interruptores y tarjetas RFID.
Cada tag tiene un valor numerico que Unity reconoce y asigna a un boton en el juego.
Cada tarjeta puede ser accionada en Unity para ingresar a una nueva escena (video). De esta forma se pueden interconectar multiples videos, 
los cuales pueden presentarse en loop, es decir no progresan hasta que el usuario ejecute una nueva opción, o con un limite de tiempo,
si el usuario no ejecuta ninguna acción, se ejecuta una de ellas por defecto.
