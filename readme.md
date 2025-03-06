<br />
<div align="center">
  <a href="https://github.com/Danmunozbe/Practica1/tree/Pain2">
    <img src="Figuras/UNAL.jpg" alt="Logo" width="200">
  </a>

  <h3 align="center">PROYECTO DE ROBÓTICA: TELEOPERACIÓN DEL ROBOT PHANTOM X PINCHER</h3>

  <p align="center">Robótica [2016770]
    <br />Andres Y. Romero D. · Luis E. Carmona A. · Miguel A. Parrado P.
    <br /> ayromerod@unal.edu.co · lucarmonaa@unal.edu.co · mparradop@unal.edu.co
  </p>
</div>


### 1. CÁLCULO DE LA CINEMÁTICA DIRECTA:

El primer paso que se realizó, para poder desarrollar la cinemática directa del Robot PhantomX Pincher, fue determinar la posición home del mismo (todas las articulaciones en cero) y tomar las medidas de cada uno de sus eslabones. En la siguiente imagen se puede observar esta información:

<p align="center">
  <img src="Figuras/dimensionesPincher.png" alt="Descripción" width="400" height="600">
</p>

El marco de referencia "base" se puso en el piso y el "TCP" se puso 15 mm antes del extremo del robot, en el punto de agarre óptimo de la pinza.

Posterior a ello, se realizó el diagrama del robót para poder dibujar los sistemas de referencia de cada eslabón siguiendo las reglas del algoritmo de Denavit-Hartenberg. Este proceso se ilustra en la siguiente figura:

<p align="center">
  <img src="Figuras/diagramaDH_pincher.png" alt="Descripción" width="600" height="400">
</p>

Observe que hay un sistema de referencia "noa" adicional a los exigidos por el algoritmo de Denavit-Hartenberg. Esto se debe a que, para que la pinza cumpla el estandar "noa", es necesario realizar una rotación adicional sobre el último sistema de referencia de Denavit-Hartenberg. Sin embargo, este último sistema de referencia "noa", al no hacer parte del algoritmo de Denavit-Hartenberg, no hará parte de la lista de los parametros DH. Teniendo en cuenta esto, a partir de la imagen anterior, es posible obtener los parametros de DH como se muestra a continuación:

<p align="center">
  <img src="Figuras/parametrosDH.png" alt="Descripción" width="800" height="150">
</p>

Teniendo en cuenta el cuadro anterior, se definen las matrices de DH y se encuentra la matriz de transformación homogenea que define la pose de "noa" respecto al sistema base. Este proceso fue realizado implementando una función en Matlab, la cual se muestra a continuación:

```matlab
function T = getDir(q) %T es la matriz de transformación que relaciona al sistema "noa" con el sistema "base" y "q" es el vector de valores articulares [q1 q2 q3 q4] en grados
    
    q = deg2rad(q); %se convierten los valores articulares a radianes
    
    % Matriz A
    A = [cos(q(1)), 0, -sin(q(1)), 0;
         sin(q(1)), 0,  cos(q(1)), 0;
         0,       -1,       0,   137;
         0,        0,       0,     1];
    
    % Matriz B
    B = [ sin(q(2)),  cos(q(2)), 0,  105*sin(q(2));
         -cos(q(2)),  sin(q(2)), 0, -105*cos(q(2));
          0,          0,        1,  0;
          0,          0,        0,  1];
    
    % Matriz C
    C = [cos(q(3)), -sin(q(3)), 0,  105*cos(q(3));
         sin(q(3)),  cos(q(3)), 0,  105*sin(q(3));
         0,         0,         1,  0;
         0,         0,         0,  1];
    
    % Matriz D
    D = [cos(q(4)), -sin(q(4)), 0,  95*cos(q(4));
         sin(q(4)),  cos(q(4)), 0,  95*sin(q(4));
         0,         0,         1,  0;
         0,         0,         0,  1];
    
    % Matriz E
    E = [0,  0,  1,  0;
        -1,  0,  0,  0;
         0, -1,  0,  0;
         0,  0,  0,  1];
    
    T = A*B*C*D*E;
end
```

Observe que la matriz E no es una matriz de DH. Despúes de realizar el alrgoritmo de DH, la matriz E se multiplica con la matriz de transformación homogenea del sistema 4 respecto a la base, para que de ese modo, las coordenadas del efector final cumplan con el estandar "noa".

A modo de ilustración se va a calcular usando Matlab la pose del efector final "noa" con un conjunto de valores articulares arbitrarios, como por ejemplo [15 30 45 60].

```matlab
>> T = getDir([15 30 45 60])
```
```plaintext
T =

    0.6830    0.2588    0.6830  213.5636
    0.1830   -0.9659    0.1830   57.2242
    0.7071         0   -0.7071  187.9335
         0         0         0    1.0000
```

### 2. CINEMÁTICA INVERSA:



### 3. DIAGRÁMA DE FLUJO:



### 4. PLANO DE PLANTA:



### 5. CÓDIGO:



### 6. TELEOPERACIÓN MANUAL - OPERACIÓN AUTOMÁTICA:



### 7. VIDEOS:

