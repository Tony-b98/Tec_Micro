clear;
clc;
close all;

% =========================================================
% CONFIGURACION SERIAL
% =========================================================

puerto = "COM4";
baudios = 9600;

s = serialport(puerto, baudios);

configureTerminator(s, "LF");
flush(s);

disp("==========================================");
disp(" CONTROL DE ILUMINACION - PROBLEMA B");
disp("==========================================");
disp("Puerto: " + puerto);
disp("Baudios: " + baudios);
disp("Leyendo datos...");
disp("Presione Ctrl+C para detener.");
disp(" ");

% =========================================================
% FIGURA UNICA
% =========================================================

fig = figure;

ax = axes(fig);

hold(ax, 'on');

% Setpoint - ROJO
hSP = animatedline( ...
    ax, ...
    'DisplayName','Setpoint', ...
    'LineWidth',2, ...
    'Color','r');

% LDR - AZUL
hLDR = animatedline( ...
    ax, ...
    'DisplayName','LDR', ...
    'LineWidth',2, ...
    'Color','b');

% Posicion - VERDE
hPOS = animatedline( ...
    ax, ...
    'DisplayName','Posicion', ...
    'LineWidth',2, ...
    'Color','g');

% PWM - MAGENTA
hPWM = animatedline( ...
    ax, ...
    'DisplayName','PWM', ...
    'LineWidth',2, ...
    'Color','m');

% =========================================================
% CONFIGURACION DE LA GRAFICA
% =========================================================

xlabel(ax,'Numero de muestra');

ylabel(ax,'Valor');

title(ax,'Control de iluminacion - Problema B');

ylim(ax,[0 1023]);

xlim(ax,[0 100]);

grid(ax,'on');

legend(ax,'show');

% =========================================================
% CONTADOR DE MUESTRAS
% =========================================================

n = 0;

% =========================================================
% FORMATO RECIBIDO POR UART
% =========================================================
%
% SP: 500 | LDR: 427 | Pos: 350 | PWM: 220 | Estado: ABRIENDO
%
% =========================================================

patron = ...
    'SP:\s*(\d+)\s*\|\s*LDR:\s*(\d+)\s*\|\s*Pos:\s*(\d+)\s*\|\s*PWM:\s*(\d+)';

% =========================================================
% LECTURA CONTINUA
% =========================================================

try

    while isvalid(fig)

        % Leer linea UART
        linea = readline(s);

        % Mostrar tambien en Command Window
        disp(linea);

        % Extraer SP, LDR, posicion y PWM
        datos = regexp( ...
            linea, ...
            patron, ...
            'tokens');

        if ~isempty(datos)

            valores = datos{1};

            SP  = str2double(valores{1});
            LDR = str2double(valores{2});
            POS = str2double(valores{3});
            PWM = str2double(valores{4});

            % Aumentar contador
            n = n + 1;

            % =================================================
            % AGREGAR DATOS
            % =================================================

            addpoints(hSP, n, SP);
            addpoints(hLDR, n, LDR);
            addpoints(hPOS, n, POS);
            addpoints(hPWM, n, PWM);

            % =================================================
            % DESPLAZAMIENTO AUTOMATICO
            % =================================================

            if n > 100

                xlim(ax,[n-100 n]);

            end

            % Actualizar grafica
            drawnow limitrate;

        end

    end

catch ME

    disp(" ");
    disp("Programa detenido.");

    if ~strcmp(ME.identifier,'MATLAB:class:InvalidHandle')
        disp(ME.message);
    end

end

% =========================================================
% CERRAR PUERTO SERIAL
% =========================================================

clear s;

disp("Puerto serial cerrado.");