# Cinedrive
UPDATE 23 NOVIEMBRE
Códigos actuales:
1-Código RR Controller MASTER: Código para convertir el arduino en un gamepad. Este es el código principal, aquí deben ir integrados todos.

2-MP3 Serial Player: Código del modulo reproductor MP3. Al accionar un botón, interruptor o tarjeta, se acciona una luz RGB y un archivo especifico de sonido almacenado en la tarjeta micro sd
*IMPORTANTE:Hay un proyecto hecho en el programa XOD IDE (Sonido y luces Astory 3.xodball), que pude cargar al Arduino que ya logra activar las luces y sonidos presionando un pulsador o interruptor. La idea sería hacer lo mismo pero en Arduino IDE con el código de YX5300 Mp3 serial Player.
Si te sirve también te dejo el código que XOD permite exportar que supuestamente sirve en Arduino IDE (Sonido_y_luces_Astory_3_importado_desde_XOD_a_Arduino_IDE.ino)

3-Lector RFID (ReadTag): falta integrar con YX5300 Mp3 Serial Player. Al reconocer una tarjeta debe accionar una luz RGB y un archivo especifico de sonido almacenado en la tarjeta.

4-Rotary Led: Código de encoder rotatorio (perilla) y luces RGB. Al girar la perilla se controlan los leds del aro de luz. Al presionar el botón de la perilla cambia la animación de las luces. (Si se puede integrar un sonido al igual que los otros códigos mejor).



///////////////////////////////////////////////////////////////////////////// DESCRIPCION GENERAL
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

EJEMPLO DE SONIDO Y LUCES EN SOFTWARE IDE XOD.
Mediante el software XOD es posible crear un código para las luces RGB y el sonido del modulo YX5300. Este código debe ser traducido a Arduino IDE.

