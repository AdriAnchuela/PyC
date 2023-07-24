while(1)
while (contador == 21)
            mensajesonar0 = receive (sonar0);
            mensajesonar2 = receive (sonar2);
            mensajesonar5 = receive (sonar5);
            mensajesonar7 = receive (sonar7);
            rangosonar0 = mensajesonar0.Range_;
            rangosonar2 = mensajesonar2.Range_;
            rangosonar5 = mensajesonar5.Range_;
            rangosonar7 = mensajesonar7.Range_;
        %% Aunque no haya una pared a la derecha, giramos a la izquierda para continuar siguiendo la pared de la izquierda.
        if (rangosonar5 > distanciaMaximaRango5)
        %% Umbrales para condiciones de parada del robot
        x_goal = 0;
        y_goal = -2;

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
        %%GIRO 90 GRADOS A IZQUIERDA
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

        %% El robot avanza hacia adelante.
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
    %% Después de haber girado a la izquierda, sabiendo que antes de girar a la izquierda no había una
    %% pared en esa dirección (donde actualmente está a la derecha), se verifica que no haya una pared detrás
    %% del robot después de haber realizado el giro a la izquierda.
    if (rangosonar0 <= distanciaMaximaRango0) && (rangosonar2 <= distanciaMaximaRango2) && (rangosonar5 <= distanciaMaximaRango5) && (rangosonar7 > distanciaMaximaRango7)
        %% Umbrales para condiciones de parada del robot
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

        %% Aplicamos las consignas de control y giramos 180 grados.
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
        %%VAMOS DE FRENTE
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

    %% Aplicamos las consignas de control y giramos 180 grados.
    if (rangosonar5 <= distanciaMaximaRango5) && (rangosonar2 > distanciaMaximaRango2)
         %% Umbrales para condiciones de parada del robot
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
    %% Cuando no hay una pared a la izquierda ni detrás, el robot se dirige hacia la derecha,
    %% ya que no detecta una pared a la izquierda y desea seguir manteniendo una pared a la izquierda.
    if (rangosonar5 <= distanciaMaximaRango5) && (rangosonar2 <= distanciaMaximaRango2) && (rangosonar0 > distanciaMaximaRango0) && (rangosonar7 > distanciaMaximaRango7)

        %% Umbrales para condiciones de parada del robot
        x_goal = 0;
        y_goal = 2;

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
    mensajesonar0 = receive (sonar0);
    mensajesonar2 = receive (sonar2);
    mensajesonar5 = receive (sonar5);
    mensajesonar7 = receive (sonar7);
    rangosonar0 = mensajesonar0.Range_;
    rangosonar2 = mensajesonar2.Range_;
    rangosonar5 = mensajesonar5.Range_;
    rangosonar7 = mensajesonar7.Range_;
        if  (rangosonar0 > distanciaMaximaRango0) && (rangosonar2 > distanciaMaximaRango0) && (rangosonar5 >distanciaMaximaRango0) && (rangosonar7 > distanciaMaximaRango0)
            break
        end
        end
        if  (rangosonar0 > distanciaMaximaRango0) && (rangosonar2 > distanciaMaximaRango2) && (rangosonar5 > distanciaMaximaRango5) && (rangosonar7 > distanciaMaximaRango7) && (contador == 21)
            break
        end
    
    % Temporización del bucle según el parámetro establecido en r
    waitfor(r)
end

