import os
import rospy
import requests


class DDDHL_Client:
    def __init__(self, user, password, debug=False) -> None:
        self.user = user
        self.password = password
        self.post_url = (
            "http://httpbin.org/post"
            if debug
            else "http://168.119.15.247:3040/apis/Mission/mRCS/ETH/"
        )

    def post_mission_data(self, full_archive_path: str) -> bool:
        if not full_archive_path.endswith("zip"):
            rospy.logerr("The path does not point to a *.zip archive")
            return False

        if os.path.isdir(full_archive_path):
            rospy.logerr(f"Failed to find {full_archive_path}")
            return False

        file_obj = open(full_archive_path, "rb")
        files = [("files", ("files.zip", file_obj, "application/zip"))]

        res = requests.post(
            url=self.post_url, auth=(self.user, self.password), files=files
        )

        rospy.logerr(
            f"Post response: {res.text}\nReturned CODE[{res.status_code}] : {res.reason}"
        )
        return res.ok


if __name__ == "__main__":
    client = DDDHL_Client(user="giuseppe.rizzi", password="Piloting2020")
    ok = client.post_mission_data("/home/giuseppe/Downloads/files.zip")
    if not ok:
        rospy.logerr("Failed to upload mission data.")
    else:
        rospy.loginfo("Successfully uploaded the mission data.")
