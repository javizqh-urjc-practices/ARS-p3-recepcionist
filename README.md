[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/5Ocj-OtY)
# Receptionist

En esta práctica, el robot debe realizar la prueba del recepcionista (página 59 del [Reglamento](https://athome.robocup.org/wp-content/uploads/2022_rulebook.pdf)), aunque algo descafeinado.

El robot tiene 10 minutos para lo siguiente:
1. El robot parte siempre de la posición inicial, que es justo detrás de la línea de al lado de la pizarra, como se indica en clase.
2. El robot va a la puerta, y cuando haya una persona (< 1.5 metros de la puerta), le pregunta su nombre, y le indica que le acompañe.
3. El robot va hacia la zona de la fiesta, le presenta, le pregunta qué quiere beber y le indica claramente (orientándose, por ejemplo) donde hay un hueco libre donde se puede sentar.
4. El robot va a la barra y le pide al barman la bebida. Considera que ya la lleva cuando el barman le dice "Aquí tienes", o algo similar.
5. El robot va a la persona y "le entrega" la bebida.
6. Goto 2

Puntuación:
* +5 Navegar hasta la puerta
* +5 Detecta persona correctamente
* +5 Navega a la zona de la fiesta
* +5 Le presenta con su nombre
* +5 Le indica correctamente donde sentarse
* +5 Pedir la bebida
* +5 Servir la bebebida

La nota total de la práctica será en función de los puntos: notas = puntos / 40.0
Habrá hasta dos puntos extra en la nota final por hacer un póster del equipo.

***

# Receptionist-Forocoches V.1.0.0

*LOGO*

¡Bienvenidos al repositorio de Receptionist-Forocoches! Somos un equipo altamente cualificado y dedicado que ha trabajado arduamente en este proyecto. Este es un modelo de sistema desarrollado en ROS-2 que permite a un robot navegar hasta varios puntos concretos utilizando mapas, además de poseer la capacidad de mantener un diálogo con el usuario mediante el uso de gb-dialog, y de reconocer tanto a personas como a sillas mediante la librería darknet-ros.

Este proyecto es el resultado de la conjunción de percepción, navegación mediante mapas y diálogo. Todas estas partes se han mejorado y perfeccionado para que el robot pueda, partiendo desde el punto inicial correcto y utilizando un mapa previamente creado, pueda navegar hacia varios puntos especificados, todo ello mientras va preguntando al usuario preguntas concretas como el nombre, su bebida favorita, e incluso la capacidad de ofrecerle sitio en un escenario que se ha representado como un bar.

*VÍDEO DEL KOBUKI EN MOVIMIENTO (LUNES)*

## Instalación

Este repositorio contiene todos los componentes necesarios para utilizar este modelo de sistema en tu propio robot. A continuación se detallan los pasos para facilitar todo lo posible la instalación y el uso de este paquete:

1. Clona el repositorio principal:

```sh
git clone https://github.com/<usuario>/<repositorio>.git
cd receptionist-forocoches
```

2. Ejecuta el instalador automático

```sh
./setup.sh
```

Opcionalmente también puedes instalar manualmente el repositorio si ya posees una o varias dependencias dentro de tu equipo. Para ello puedes utilizar el siguiente comando para seleccionar manualmente las instalaciones necesarias:

```sh
vcs import third_parties.repos
```

Recuerda que es importante desargar de forma recursiva la dependencia de darknet, y una vez se haya descargado, se deben instalar dichos paquetes en la terminal que se esté utilizando.

Toda está información se puede encontrar de forma más amplia en: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html

Una vez se haya instalado el repositorio, podrás poner en marcha tu robot y hacer que vaya navegando por los puntos especificados mientras le va haciendo preguntas al usuario en momentos concretos para que éste le responda de vuelta, además de poder reconocer a la persona cuando ésta llega a la puerta, o cuando una silla está vacía y el robot pueda ofrecerle sitio.

## Modo de uso

Para lanzar el paquete Receptionist-Forocoches en ROS-2, es necesitario utilizar 2 launchers. Un launcher llamado dependencies.launch.py, que incluye todas las dependencias que hay que lanzar; y otro llamado receptionist_forocoches.launch.py, en el cual se incluye el ejecutable completo del paquete y que puede lanzarse tantas veces como se solciite.

Además, es importante asegurarse de conectar los correspondientes paquetes de sincronización con el robot Kobuki para que el sistema puede recibir y enviar datos correctamente.

```sh
ros2 launch receptionist-forocoches dependencies.launch.py
ros2 launch receptionist-forocoches receptionist_forocoches.launch.py
```

## Comportamiento (Percepción)

*PENDIENTE DE PRUEBA (LUNES)*

## Comportamiento (Diálogo)

*PENDIENTE DE PRUEBA (LUNES)*

## Comportamiento (Behavior Tree)

*PENDIENTE DE PRUEBA (LUNES)*

## Autores
* Javier Izquierdo
* Alberto León
* Luis Moreno
* Sebastian Mayorquín

## Contribuciones

Las contribuciones son bienvenidas. Si deseas contribuir a este proyecto, por favor, crea un pull request. Asegúrate de seguir las directrices de contribución antes de hacerlo.

__By Forocoches__

