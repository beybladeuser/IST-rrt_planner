import gdown
import os

def download_rosbag(file_url, output_path):
	print(f"Downloading rosbag {output_path}")
	if os.path.exists(output_path):
		print(f"Bag already downloaded in {output_path}")
	else:
		gdown.download(file_url, output_path, quiet=False)
		print("Download successful")

if __name__ == '__main__':
	files = [
		{
			"file_url": 'https://drive.google.com/uc?export=download&id=1RxKFzekNP2pBH9KPIxtlNRbTQVpYcOB2',
			"output_path": 'rosbag_recording_lab_correct_path.bag'
		}
	]
	for file in files:
		download_rosbag(file["file_url"], file["output_path"])