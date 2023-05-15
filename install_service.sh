sudo cp puara-serial-manager.service /etc/systemd/system/puara-serial-manager.service
sudo chmod 664 /etc/systemd/system/puara-serial-manager.service
sudo systemctl daemon-reload
sudo systemctl enable --now puara-serial-manager.service
