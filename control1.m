%% Bucle de Control infinitopsalida
%% Saber cuantas paredes hay en los sonares
while (1)
    b=false;
    mensajesonar0 = receive (sonar0);
    mensajesonar2 = receive (sonar2);
    mensajesonar5 = receive (sonar5);
    mensajesonar7 = receive (sonar7);
    %% Se observan las distancias medidas por los sensores de sonar con
    %% respecto a las cuatro paredes.

    %% SONAR QUE APUNTA A PARED IZQUIERDA DEL ROBOT
    rangosonar0 = mensajesonar0.Range_;

    %% SONAR QUE APUNTA PARED DE DELANTE DEL ROBOT
    rangosonar2 = mensajesonar2.Range_;

    %% SONAR QUE APUNTA A PARED DERECHA DEL ROBOT
    rangosonar5= mensajesonar5.Range_;

    %% SONAR QUE APUNTA PARED DE DETRAS DEL ROBOT
    rangosonar7 = mensajesonar7.Range_;



    
    %% Se analizan todos los posibles casos de casillas con sus correspondientes
    %% movimientos del robot, guardando las posiciones por las que pasa para
    %% verificar que cubre todas ellas. Este procedimiento implica utilizar
    %% los diferentes sensores del robot para medir el rango en el centro de
    %% cada casilla y compararlo con la distancia máxima establecida.
    %% Este código es similar al utilizado en el archivo "apartado2_8.m"
    %% de la práctica 1.
    
    if (rangosonar0 > distanciaMaximaRango0)
    %% En caso de que la medida de alcance del robot sea mayor que 1.1 (que
    %% es nuestro valor de distancia máxima de rango), se lleva a cabo el
    %% siguiente procedimiento:
    
        %% Umbrales para condiciones de parada del robot
        %% Siempre avanzamos de 2 en 2 para desplazarnos una casilla.
        %% Si queremos movernos una casilla a la derecha del robot, utilizamos
        %% los siguientes comandos para que el robot gire sobre sí mismo 90
        %% grados en sentido horario, de modo que su eje y se convierta en su
        %% eje x. Una vez reorientado, como veremos más adelante, le
        %% indicamos que se mueva una casilla hacia adelante.
        x_goal = 0;
        y_goal = 2;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a POSICION RELATIVA a donde esta el robot en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
         %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;
        %% Comprobar que E_ori siempre tenga un valor comprendido en el
        %% intervalo [-pi,pi]
        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist * 0.1);
        %% La constante de proporcionalidad es de 0.1 y es importante
        %% asegurarse de que nuestra velocidad lineal no sea mayor que 1.
        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end
        %% La constante de proporcionalidad es de 2.
        consigna_vel_ang = Eori * 2;
        %% Nuestra velocidad lineal no debe ser mayor que 1.
        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        %% Si el error de orientación no supera el umbral establecido,
        %% no es necesario corregir ese error.
        if (abs(Eori)<umbral_angulo)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break; 
        end
        %% Aplicamos consignas de control
        %% El robot gira sobre su propio eje para orientarse de tal
        %% forma que pueda subir una casilla hacia arriba.
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= consigna_vel_ang;
        % Comando de velocidad
        send(pub,msg_vel);
        end
        
        %% Umbrales para condiciones de parada del robot
        %% Una vez que nos hemos reorientado para que el eje y del robot
        %% pase a ser su eje x, nos movemos hacia adelante una casilla,
        %% como se mostrará a continuación.
        x_goal = 2;
        y_goal = 0;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a posición relativa a donde esta
        %% el robot en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;
        %% Comprobar que E_ori siempre tenga un valor comprendido en
        %% el intervalo [-pi,pi]
        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
        %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist);
        %% Nuestra velocidad lineal no debe ser mayor que 1.
        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 0.5;
        %% La constante de proporcionalidad es de 0.5 y es importante
        %% asegurarse de que nuestra velocidad angular no sea mayor que 1.
        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        %% Si el error de distancia, en este caso, al utilizar la velocidad lineal,
        %% no supera el umbral establecido, no es necesario corregir ese error.
        if (Edist<umbral_distancia)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end
        %% Aplicamos consignas de control
        %%NOS MOVEMOS DE FRENTE UNA CASILLA
        msg_vel.Linear.X= consigna_vel_linear;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        % Comando de velocidad
        send(pub,msg_vel);
        end
      
    end



    %% A partir de este punto, procederemos a verificar, de manera similar a como
    %% lo hicimos en la práctica 1, en qué lados hay paredes y en qué lados no
    %% las hay. En este caso en particular, no se detecta una pared detrás del robot.
    if (rangosonar0 <= distanciaMaximaRango0) && (rangosonar2 <= distanciaMaximaRango2) && (rangosonar5 <= distanciaMaximaRango5) && (rangosonar7 > distanciaMaximaRango7)

        %% Umbrales para condiciones de parada del robot
        %% Por lo tanto, nuestro objetivo será desplazarnos a lo largo del eje x
        %% del robot en ese momento, retrocediendo una casilla. Como
        %% veremos a continuación, esto se logrará girando primero 180 grados
        %% y luego avanzando una casilla hacia atrás.
        x_goal = -2;
        y_goal = 0;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %Actualizamos el destino a posición relativa a donde esta el robot en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;
        %% Comprobar que E_ori siempre tenga un valor comprendido en el intervalo [-pi,pi]
        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist * 0.1);
        %% Nuestra velocidad lineal no debe ser mayor que 1.
        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 2;
        %% Nuestra velocidad angular no debe ser mayor que 1.
        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        %% Si el error de orientación no supera el umbral establecido en este caso,
        %% donde giramos sobre el propio eje del robot, no es necesario corregir ese error.
        if (abs(Eori)<umbral_angulo)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end
        %%En este caso, el robot realizará un giro de 180 grados primero y
        %% luego avanzará hacia adelante, saliendo de la situación concreta
        %% que estamos evaluando en esta condición "if".
        %% Aplicamos consignas de control
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= consigna_vel_ang;

        % Comando de velocidad
        send(pub,msg_vel);
        end
        %% Umbrales para condiciones de parada del robot
        %% Una vez que hemos girado, nuestro objetivo cambia, ya que al avanzar
        %% hacia adelante después de haber girado 180 grados, ya podemos
        %% salir de la situación actual.
        x_goal = 2;
        y_goal = 0;
        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);
        %Actualizamos el destino a posición relativa a donde esta el robot en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;
        %% Comprobar que E_ori siempre tenga un valor comprendido en el intervalo [-pi,pi]
        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
        %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist);
        %% Nuestra velocidad lineal no debe ser mayor que 1.
        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 0.5;
        %% Nuestra velocidad angular no debe ser mayor que 1.
        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        %% Si el error de distancia en este caso, al movernos linealmente después,
        %% no supera el umbral establecido, no se realiza ninguna corrección de
        %% ese error.
        if (Edist<umbral_distancia)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end        
        %% Aplicamos consignas de control
        %% Nos movemos hacia adelante una casilla.
        msg_vel.Linear.X= consigna_vel_linear;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        % Comando de velocidad
        send(pub,msg_vel);
        end
    end





    %% Si hay una pared a la izquierda y no hay una pared enfrente, avanzamos
    %% una casilla hacia adelante directamente, como se mostrará a continuación.
    if (rangosonar0 <= distanciaMaximaRango0) && (rangosonar2 > distanciaMaximaRango2)
        %% Umbrales para condiciones de parada del robot
        x_goal = 2;
        y_goal = 0;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a posición relativa a donde esta el robot en
        %% el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;

        %% Comprobar que E_ori siempre tenga un valor comprendido en el intervalo
        %[-PI,PI]
        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end

         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist);
        %%NO MAYOR DE 1 VELOCIDAD LINEAL
        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 0.5;
        %% La velocidad angular no debe ser mayor que 1.
        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        %% Si el error no supera el umbral establecido, no se realiza
        %% ninguna corrección de error.
        if (Edist<umbral_distancia)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end
        %% Aplicamos consignas de control

        %% Avanzamos una casilla hacia adelante.
        msg_vel.Linear.X= consigna_vel_linear;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        % Comando de velocidad
        send(pub,msg_vel);
        end
    end




    %% Si no hay una pared a la derecha y tampoco una pared en la parte trasera,
    %% el robot girará 90 grados a la izquierda y luego se moverá una casilla
    %% hacia adelante.
    if (rangosonar0 <= distanciaMaximaRango0) && (rangosonar2 <= distanciaMaximaRango2) && (rangosonar5 > distanciaMaximaRango5) && (rangosonar7 > distanciaMaximaRango7)
        %% Umbrales para condiciones de parada del robot
        x_goal = 0;
        y_goal = -2;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a posición relativa a donde esta el
        %% robot en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;

        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist * 0.1);

        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 2;

        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        if (abs(Eori)<umbral_angulo)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end

        %% Aplicamos consignas de control
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= consigna_vel_ang;
        % Comando de velocidad
        send(pub,msg_vel);
        end
        
        %% Umbrales para condiciones de parada del robot
        x_goal = 2;
        y_goal = 0;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a posición relativa a donde esta el
        %% robot en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;
        %%Comprobar que E_ori siempre
        % tenga un valor comprendido en el intervalo [-π,π]
        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist);

        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 0.5;

        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end

        %% Si el error no supera el umbral establecido, no se realiza
        %% corrección de error.
        if (Edist<umbral_distancia)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end

        %% Aplicamos consignas de control
        msg_vel.Linear.X= consigna_vel_linear;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        % Comando de velocidad
        send(pub,msg_vel);
        end
       
    end
    
    mensajesonar0 = receive (sonar0);
    mensajesonar2 = receive (sonar2);
    mensajesonar5 = receive (sonar5);
    mensajesonar7 = receive (sonar7);

    rangosonar0 = mensajesonar0.Range_;
    rangosonar2 = mensajesonar2.Range_;
    rangosonar5 = mensajesonar5.Range_;
    rangosonar7 = mensajesonar7.Range_;
 

    
    %% Si estamos fuera y el valor del contador es menor que 21 casillas,
    %% decidimos volver girando 180 grados y luego avanzar una casilla hacia
    %% adelante. De esta manera, podemos dirigirnos a las casillas restantes
    %% que aún necesitamos analizar.
     if (rangosonar0 > distanciaMaximaRango0) && (rangosonar2 > distanciaMaximaRango2) && (rangosonar5 > distanciaMaximaRango5) && (rangosonar7 > distanciaMaximaRango7) && (contador < 21)
         pos = odom.LatestMessage.Pose.Pose.Position;
         Xsalida=round(pos.X);
         Ysalida=round(pos.Y);
         ArraySalida = [Xsalida Ysalida];
         %% Umbrales para condiciones de parada del robot
        x_goal = -2;
        y_goal = 0;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a posición relativa a donde esta el robot en
        %% el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
         %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;

        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist * 0.1);

        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 2;

        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        if (abs(Eori)<umbral_angulo)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end

        %% Aplicamos consignas de control
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= consigna_vel_ang;

        % Comando de velocidad
        send(pub,msg_vel);
        end
        %% Umbrales para condiciones de parada del robot
        x_goal = 2;
        y_goal = 0;

        %% Obtenemos la posición y orientación INICIAL
        pos_ini=odom.LatestMessage.Pose.Pose.Position;
        ori_ini=odom.LatestMessage.Pose.Pose.Orientation;
        yaw_ini=quat2eul([ori_ini.W ori_ini.X ori_ini.Y ori_ini.Z]);
        yaw_ini=yaw_ini(1);

        %% Actualizamos el destino a posición relativa a donde esta el robot
        %% en el momento inicial de ejecutar este script.
        x_goal_new = (pos_ini.X + x_goal*cos(yaw_ini) - y_goal*sin(yaw_ini));
        y_goal_new = (pos_ini.Y + x_goal*sin(yaw_ini) + y_goal*cos(yaw_ini));
        
        while  (1)
        %% Obtenemos la posicion y orientacion actuales
        pos = odom.LatestMessage.Pose.Pose.Position;
        ori = odom.LatestMessage.Pose.Pose.Orientation;
        yaw = quat2eul([ori.W ori.X ori.Y ori.Z]);
        yaw = yaw(1);

        %% Calculamos el error de distancia
        Edist= sqrt((pos.X-x_goal_new)^2+(pos.Y-y_goal_new)^2);

        %% Calculamos el error de orientacion 
        Eori= (atan2((y_goal_new-pos.Y),(x_goal_new-pos.X)))-yaw;

        if Eori<-pi
            Eori=Eori+(2*pi);
        end
        if Eori>pi
            Eori=Eori-(2*pi);
        end
         %% Calculamos las consignas de velocidades
        consigna_vel_linear = (Edist);

        if consigna_vel_linear > 1
            consigna_vel_linear = 1;
        end

        consigna_vel_ang = Eori * 0.5;

        if consigna_vel_ang > 1
            consigna_vel_ang = 1;
        end
        if (Edist<umbral_distancia)
        msg_vel.Linear.X= 0;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        send(pub,msg_vel);
        break;
        end
        %% Aplicamos consignas de control
        msg_vel.Linear.X= consigna_vel_linear;
        msg_vel.Linear.Y=0;
        msg_vel.Linear.Z=0;
        msg_vel.Angular.X=0;
        msg_vel.Angular.Y=0;
        msg_vel.Angular.Z= 0;
        % Comando de velocidad
        send(pub,msg_vel);
        end
     end
        %% Al final del archivo practica3.m, agregamos código para guardar en un
        %% arreglo todas las casillas por las que ha pasado el robot y sumar 1
        %% al contador por cada casilla guardada.

        pos = odom.LatestMessage.Pose.Pose.Position;

        %% Se verifica si las posiciones por las que pasa el robot ya han sido
        %% recorridas o no. Si no han sido recorridas, se guardan las nuevas
        %% posiciones en el arreglo. Todo esto se repite hasta que el contador
        %% alcance el valor de 25, lo cual significa que se han recorrido las 25
        %% casillas.

        X = round(pos.X);
        Y = round(pos.Y);
        for i = 1:contador 
                if (ArrayX(i) == X) && (ArrayY(i) == Y)
                    b = true;
                end
        end
        if (~b)
            ArrayX(end+1)= X;
            ArrayY(end+1)= Y;
            contador = contador + 1;
        end
        if (contador==25)
            break;
        end
        
end    
