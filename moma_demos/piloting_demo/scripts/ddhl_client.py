import os
import rospy
import requests

from requests_toolbelt.multipart.encoder import MultipartEncoder

mp_encoder = MultipartEncoder(
    fields={
        # Content-Disposition header with just the part name
        "mission": open(
            "/home/giuseppe/Downloads/files.zip", "rb"
        )  # ('files.zip', open("/home/giuseppe/Downloads/files.zip", 'rb'), 'multipart/form-data'),
    }
)


class DDDHL_Client:
    def __init__(self, debug=False) -> None:
        self.user = "giuseppe.rizzi"
        self.password = "Piloting2020"
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
        res = requests.post(
            url=self.post_url,
            auth=(self.user, self.password),
            # headers = { 'Content-Type' : 'multipart/form-data' },
            files={"file": file_obj},
        )  # ("files", file_obj)})
        # data=mp_encoder)
        rospy.loginfo(res)
        rospy.logerr(f"Response returned {res}")
        return res.ok


if __name__ == "__main__":
    client = DDDHL_Client()
    ok = client.post_mission_data("/home/giuseppe/Downloads/files.zip")
    if not ok:
        rospy.logerr("Failed to upload mission data.")
    else:
        rospy.loginfo("Successfully uploaded the mission data.")
