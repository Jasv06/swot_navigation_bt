Anleitung für den Betrieb des mobilen Roboters

1) Starten des Evocortex

	1. Zugriff auf den Evocortex mittels SSH
        	ssh irobot@192.168.0.2
    	pw: myrobot

	1.1 Initialisierung der CAN-Schnittstelle
        	sudo ~/ws/c*/src/evo*/evo*exam*/scripts/can_init.sh

	1.2 Laden der notwendigen Treiber (für Motorcontroller, SICK-Laser etc.)
        	roslaunch swot_bringup bringup.launch


2) Starten des TimeSync (Zeitsynchronisation zwischen Upboard und Evocortex)

    	2.1 Zugriff auf das Upboard mittels SSH
        	ssh fe-w364@192.168.0.3
        	pw: myrobot

    	2.2 Starten des NTP-Servers
        	sudo systemctl restart ntp

   	2.3 Zugriff auf den Evocortex mittels SSH
        	ssh irobot@192.168.0.2
    	pw: myrobot

    	2.4 Starten des NTP-Client
        	sudo systemctl daemon-reload 
        	sudo systemctl restart systemd-timesyncd 
        	sudo systemctl status systemd-timesyncd
        	(Sync erfolgreich, wenn status: Synchronized to 192.168.0.3)


3. Anleitung für das Starten der Navigation

    	3.1 Zugriff auf das Upboard mittels SSH
        	ssh fe-w364@192.168.0.3
        	pw: myrobot

    	3.2 Roboter an Startpose bringen (Bodenmarkierung)

    	3.3 Starten der Navigation
        	roslaunch swot_navigation swot_robocup_navigation.launch

