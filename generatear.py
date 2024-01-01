import cv2 
import cv2.aruco as aruco
import argparse
import numpy as np 
import os 


class CreateAruco:
	def __init__(self,aruco_dict,marker_pixel,border_pixel,marker_size,path_to_save):
		self.marker_id = 0
		self.marker_image = None
		self.aruco_dict = aruco_dict
		self.marker_pixel = marker_pixel
		self.border_pixel = border_pixel
		self.marker_size = marker_size
		self.path_to_save = path_to_save
		self.make_dir(self.path_to_save)

	def make_dir(self,path):
		try:
			os.makedirs(path)
		except OSError:
			pass

	def create_and_save_a_marker(self,marker_id):
		self.marker_id = marker_id
		self.make_dir(self.path_to_save+"/tag_"+str(self.marker_id)+"/materials/textures")
		self.make_dir(self.path_to_save+"/tag_"+str(self.marker_id)+"/materials/scripts")
		self.create_marker()
		self.save_model_image()
		self.save_model_config()
		self.save_model_materials()
		self.save_model_sdf()
		print("create and save marker:",self.marker_id)

	def create_and_save_multi_marker(self,marker_id_list):
		for marker_id in marker_id_list:
			self.create_and_save_a_marker(marker_id)

	def create_marker(self):
		image = aruco.drawMarker(aruco_dict, self.marker_id, self.marker_pixel)
		self.marker_image = cv2.copyMakeBorder(image, 
											self.border_pixel,
											self.border_pixel,
											self.border_pixel,
											self.border_pixel,
											cv2.BORDER_CONSTANT,
											None, [255,255,255])

	def save_model_image(self):
		cv2.imwrite(self.path_to_save+"/tag_"+ \
					str(self.marker_id)+"/materials/textures/aruco_marker_"+ \
					str(self.marker_id)+".png", self.marker_image)

	def save_model_materials(self):
		file = open(self.path_to_save+"/tag_"+str(self.marker_id)+"/materials/scripts/tag.material", 'w')

		file.write("\n \
			material aruco_tag_"+str(self.marker_id)+"\n \
			{\n \
				technique\n \
				{\n \
				pass\n \
				{\n \
					texture_unit\n \
					{\n \
						// Relative to the location of the material script\n \
						texture ../textures/aruco_marker_"+str(self.marker_id)+".png\n \
						filtering anisotropic\n \
						max_anisotropy 16\n \
					}\n \
				}\n \
				}\n \
			}\n")
		file.close()

	def save_model_config(self):
		file = open(self.path_to_save+"/tag_"+str(self.marker_id)+"/model.config", 'w')
		file.write("\n \
			<?xml version=\"1.0\"?>\n \
			\n \
			<model>\n \
			<name>Aruco tag"+ str(self.marker_id) +"</name>\n \
			<version>1.0</version>\n \
			<sdf version=\"1.6\">model.sdf</sdf>\n \
			\n \
			<author>\n \
				<name>realjc</name>\n \
				<email>2904152204@qq.com</email>\n \
			</author>\n \
			\n \
			<description>\n \
			Aruco tag "+str(self.marker_id)+"\n \
			</description>\n \
			\n \
			</model>\n \
					")
		file.close()


	def save_model_sdf(self):
		file = open(self.path_to_save+"/tag_"+str(self.marker_id)+"/model.sdf", 'w')
		file.write("\n \
			<?xml version=\"1.0\"?>\n \
			<sdf version=\"1.6\">\n \
			<model name=\"Aruco tag"+ str(self.marker_id) + "\">\n \
				<static>true</static>\n \
				<link name=\"robot_link\">\n \
          <visual name=\"front_visual\">\n \
            <pose>"+str(marker_size[0]/2)+" "+str(0)+" "+str(0)+" "+str(0)+" "+str(0)+" "+str(0)+" "+"</pose>\n \
              <geometry>\n \
                <box>\n \
                  <size>"+str(marker_size[0])+" "+str(marker_size[1])+" "+str(marker_size[2])+" "+"</size>\n \
                </box>\n \
              </geometry>\n \
              <material> <!-- Body material -->\n \
                <script>\n \
                  <uri>model://tag_"+str(self.marker_id)+"/materials/scripts/tag.material</uri>\n \
                  <name>aruco_tag_"+str(self.marker_id)+"</name>\n \
                </script>\n \
              </material> <!-- End Body Material -->\n \
          </visual>\n \
          <visual name=\"rear_visual\">\n \
            <pose>"+str(-marker_size[0]/2)+" "+str(0)+" "+str(0)+" "+str(0)+" "+str(0)+" "+str(0)+" "+"</pose>\n \
              <geometry>\n \
                <box>\n \
                  <size>"+str(marker_size[0])+" "+str(marker_size[1])+" "+str(marker_size[2])+" "+"</size>\n \
                </box>\n \
              </geometry>\n \
          </visual>\n \
				</link>\n \
			</model>\n \
			</sdf>\n \
			\n")
		file.close()


if __name__ == "__main__":
	aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
	marker_pixel = 900
	marker_size = [0.005,0.33,0.33]
	border_pixel = 100
	path_to_save = "./models"
	aruco_gazebo = CreateAruco(aruco_dict,marker_pixel,border_pixel,marker_size,path_to_save)
	aruco_gazebo.create_and_save_multi_marker([i for i in range(30)])
