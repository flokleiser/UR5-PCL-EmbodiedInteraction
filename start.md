docker compose up -d ursim viewer
docker compose run --rm -p 8765:8765 dev bash
python3 students/flo/tonspur/server.py
