cat <<EOF > GUIDE.md
# Kvik-guide: Sådan starter du TurtleBot3 (ROS 2 Humble)

Denne guide forudsætter, at robotten er tændt, og du har forbindelse til WiFi.
Vigtigt: Du skal bruge 2 separate terminal-vinduer.

## Trin 1: Tænd for hardwaren
1. Tjek at batteriet er sat i (eller strømforsyning til OpenCR-boardet).
2. Tænd på kontakten på siden af robotten.
3. Vent til den spiller start-melodien (Do-Re-Mi...).

---

## Trin 2: Start robot-driveren (Vindue 1)
Åbn en terminal og log ind på robotten:
ssh ubuntu@<ROBOTTENS_IP>

Kør start-kommandoen (Bringup):
ros2 launch turtlebot3_bringup robot.launch.py

* Succes: Du ser tekst der ender med "Start Bringup", og ingen røde fejl.
* Lad dette vindue stå åbent! Det er robottens "puls".

---

## Trin 3: Kør dit program (Vindue 2)
Åbn en ny terminal (lad den første køre i baggrunden) og log ind igen:
ssh ubuntu@<ROBOTTENS_IP>

Kør dit Python-script:
python3 testniels.py (eksempel)

* Nu skal robotten køre! (Husk at gribe den).

---

## Hurtig Fejlfinding

### Robotten siger "Waiting for subscription..."
* Tjek at Vindue 1 stadig kører "ros2 launch...".
* Hvis Vindue 1 er lukket eller stoppet, kan din kode ikke snakke med hjulene.

### Fejl: "LDS_MODEL" eller "TURTLEBOT3_MODEL" missing
Hvis den har glemt, hvad den er, så kør disse kommandoer én gang i terminalen for at gemme det permanent:
echo 'export TURTLEBOT3_MODEL=burger' >> ~/.bashrc
echo 'export LDS_MODEL=LDS-01' >> ~/.bashrc
source ~/.bashrc

### Hjulene låser ikke / Robotten reagerer slet ikke
* Tjek om den røde/sorte ledning til motorerne sidder løst under robotten.
* Tryk på Reset-knappen (lille knap på OpenCR printpladen), vent på lyden, og start forfra med Trin 2.
EOF