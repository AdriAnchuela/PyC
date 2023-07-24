%% INICIALIZACION DE ROS
setenv('ROS_MASTER_URI','http://192.168.31.190:11311')
setenv('ROS_IP','192.168.31.189')
% Inicialización de ROS
rosinit 


%% DECLARACION DE VARIABLES NECESARIAS PARA EL CONTROL
%% Inicializamos los errores en 0, o, alternativamente, asumimos que comienzan en 0, similar a cómo se hizo en la práctica 1.
Edist = 0;
Eori = 0;
%% Umbrales para condiciones de parada del robot

umbral_distancia =0.0001;
umbral_angulo = 0.00001;
%% Haremos 3 arrays, de tal forma que uno guarde la salida de la posicion, y los otros recopilan las coordenadas x e y.
ArraySalida = []; 
ArrayX = [];  
ArrayY = [];   
contador = 0;
%% Array iniciador en 0,0
ArrayInicio = [0,0]; 


%% DECLARACIÓN DE SUBSCRIBERS
odom=rossubscriber('/robot0/odom'); 
laser=rossubscriber('/robot0/laser_1');
sonar0=rossubscriber('/robot0/sonar_0');
sonar1=rossubscriber('/robot0/sonar_1');
sonar2=rossubscriber('/robot0/sonar_2');
sonar6=rossubscriber('/robot0/sonar_6');
sonar3=rossubscriber('/robot0/sonar_3');
sonar4=rossubscriber('/robot0/sonar_4');
sonar5=rossubscriber('/robot0/sonar_5');
sonar7=rossubscriber('/robot0/sonar_7');

%% DECLARACION DE PUBLSHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub); %Creamos un mensaje tipo declarado en "pub" (geometry_msg/Twist)


%% Definimos la perodicidad del bucle (10 hz)
r = robotics.Rate(10);
waitfor(r);

%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0"
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1)
 odom.LatestMessage
end

%% Primero, nuestro objetivo es determinar cuántas paredes existen a lo largo del trayecto del robot. Este enfoque es similar al de la práctica 1, 
%% en la que se nos solicitaba identificar las paredes. Por lo tanto, el código tendrá similitudes con el segmento 8 relacionado con el láser de la práctica 1.

%% Recibiremos el mensaje proporcionado por el láser y definiremos la distancia máxima en rango que, en base a la experiencia de la práctica 1, 
%% estableceremos inicialmente en 1.1. Sin embargo, este valor puede requerir ajustes en caso de que no proporcione resultados óptimos.
mensajeLaser = receive (laser);




%% Estas nos serviaran mas adelante
distanciaMaximaRango0 = 1.1;
distanciaMaximaRango2 = 1.1;
distanciaMaximaRango5 = 1.1;
distanciaMaximaRango7 = 1.1;



%% Almacenamos la posición inicial del robot en la variable 'pos'.
pos = odom.LatestMessage.Pose.Pose.Position;

    %%  Continuamos incorporando las coordenadas X e Y en los arrays que hemos definido previamente.
    %%  'END' se utiliza para representar el último índice de una matriz. Por ejemplo, X(end) se refiere al último elemento de X, 
		%%  y X(3:end) selecciona los elementos desde el tercero hasta el final de X.
    %%  Tambien el end la usamos en la parte 6 de la practica 1
    ArrayX(end+1)= pos.X;
    ArrayY(end+1)= pos.Y;
    contador = contador + 1;
