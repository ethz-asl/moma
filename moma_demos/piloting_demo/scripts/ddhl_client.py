#!/usr/bin/python3

import os

import rospy
import requests
import argparse
from getpass import getpass


class DDDHL_Client:
    """
    DDHL Client Class
    Intended use: after report generation, this class can be used to post
    the report archive to the server. Requires to previously sign up and use
    the corresponding credentials for authentication
    """

    def __init__(self, user, password, debug=False) -> None:
        self.user = user
        self.password = password
        self.post_url = (
            "http://httpbin.org/post"
            if debug
            else "https://piloting-ddhl.inlecomsystems.com/apis/Mission/mRCS/ETH/"
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

        rospy.loginfo(f"Posting {full_archive_path}")
        res = requests.post(
            url=self.post_url, auth=(self.user, self.password), files=files
        )

        rospy.logerr(
            f"Post response: {res.text}\nReturned CODE[{res.status_code}] : {res.reason}"
        )
        return res.ok


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="DDHL Client.")
    parser.add_argument(
        "-i", "--input", help="full path to the report folder", required=True
    )
    parser.add_argument(
        "-u", "--user", help="username", required=False, default="julian.keller"
    )
    parser.add_argument(
        "-p", "--password", help="password", required=False, default="piloting"
    )
    args = parser.parse_args()

    if not args.user:
        args.user = input("User: ")

    if not args.password:
        args.password = getpass(prompt=f"Password for {args.user}: ")

    client = DDDHL_Client(user=args.user, password=args.password)
    ok = client.post_mission_data(args.input)
    if not ok:
        rospy.logerr("Failed to upload mission data.")
    else:
        rospy.loginfo("Successfully uploaded the mission data.")
