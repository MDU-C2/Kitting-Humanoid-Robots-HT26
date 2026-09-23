import cv2

stream_url = "http://192.168.123.164:8080/stream.mjpg"
print(f"Ansluter till roboten: {stream_url}...")

cap = cv2.VideoCapture(stream_url)

if not cap.isOpened():
    print(
        "Kunde inte ansluta till strömmen. "
        "Säkerställ att robotens skript är igång "
        "och att IP 192.168.123.164 kan nås."
    )
    exit()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Tappade bildrutor från roboten...")
        break

    cv2.imshow("Unitree G1 - Live YOLO Detection", frame)

    # Tryck 'q' för att stänga fönstret
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()