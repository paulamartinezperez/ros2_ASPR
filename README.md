# ros2_ASPR
EJERCICIO ROS2 CLASE ASPR- Ingenieria de Robotica Software URJC

La aplicación tiene 3 procesos Unix que se lanzan en un launcher.
El Proceso 1 tiene dos nodos, como muestra la figura, con los nombres de nodos y topics que figuran ahí. El nodo_A publica a 20 Hz un string que contiene “message: N”, donde N es un número de secuencia que empieza en 0 y se incremente en cada iteración. La QoS de este topic es best_effort.
El Proceso 2 tiene dos nodos, como muestra la figura, con los nombres de nodos y topics que figuran ahí. El nodo_C publica a 10 Hz un PoseStamped con coordenadas aleatorias. Su QoS es la de por defecto (la misma que en ROS). Son ambos lifecycle nodes que deben ser llevados al estado activo desde la terminal.
El proceso 3 tiene un nodo que cada segundo le pregunta tanto a nodo_B como a nodo_D por el último valor que recibieron, usando un servicio cuyo mensaje tenéis que crear en otro paquete aparte.
Los nodos deben mostrar por INFO lo que van recibiendo.
Debéis integrar tests, al menos los de estilo.
