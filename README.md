# Paramics-Omnet

Simulacion de protocolos coordinacion vehicular en intersecciones, utilizando los simuladores Paramics y Omnet-5.0, ademas de PVeins, una modificacion de Veins-4.4 que funciona en conjunto con un plugin de Paramics (https://github.com/molguin92/paramics_traci/releases/tag/v0.8-release).

# Ejemplo de uso

En este repositorio se encuentran 4 protocolos de coordinacion vehicular en intersecciones, ademas de algunas modificacion de Pveins, las cuales permiten la generacion de simulacion sin la herramienta grafica de Paramics.

Para utilizar los protocolos en simualcion:

    1. Instalar y configurar OMNeT++ 5.0 (instrucciones mas detalladas se encuentran en [carpeta con omnet++]/doc/InstallGuide.pdf)

    2. Bajar y compilar Veins-4.4 (presente en este repositorio) en el entorno bash de OMNeT++. Para ello:

        - Ejecute mingwenv.cmd (esta en la carpeta de omnet++)

        - Ir al directorio de Veins (en el entorno bash de omnet++)

        - Ejecutar "./configure && MODE=release make" (puede utilizarse MODE=debug para mostrar mensajes de debugging, pero se vuelve muy lenta la simulación)

        - Mantener el entorno bash abierta, ya que se utilizará más adelante

    3. Abrir una consola o PowerShell de Windows en la carpeta donde se encuentra paramics-launchd.py, y ejecutar

        <Dirección del ejecutable de python>\python.exe paramics-launchd.py -vv

    IMPORTANTE: El programa paramics-launchd.py se encarga de iniciar Paramics, este programa utiliza el plugin y ejecutable Modeller de Paramics, y para ello utiliza un path hacia ambos archivos. En caso de no entregarse un path, se utiliza uno por defecto, por lo que puede ocurrir problemas al ejecutar la simulacion.

    4. Luego, en la ventana del entorno bash de OMNeT++, cambiar a la carpeta veins4.4/examples/CrossRoad y ejecutar

        ./run -u Cmdenv -c nodebug

    Si todo sale bien, eso debiese ejecutar la simulación en Paramics.

Algunos puntos extras a tener en cuenta:

    1. Para este ejemplo, el archivo paramics.launchd.xml que se encuentra en veins4.4/examples/CrossRoad contiene la configuracion basica para realizar la simulacion, uno de los datos presentes en este archivo es el path hacia la red de transporte, si no se cambia por el path correcto producira errores al iniciar simulaciones.

    2. Para este ejemplo, el archivo omnet.ini contiene una configuracion avanzada para la realizacion de simulaciones. Dentro de los parametros de la configuracion, esta la eleccion del comportamiento de los vehiculos definida por los protocolos. En caso que una simulacion no realize lo esperado, se puede deber a que se esta utilizando un protocolo distinto, para ello, revise cual es el valor de: *.node[*].applType, el cual debe ser el nombre del protocolo que se esta utilizando.