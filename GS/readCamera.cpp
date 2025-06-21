//readCamrea.cpp

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

using namespace std;

void convertCameraFile(const string& input_path, const string& output_path) {

	std::unordered_map<string, string> cam_params;
	std::ifstream inputfile(input_path);

	if (!inputfile) {
		cout << "打开文件错误" << endl;
		return;
	}
	string line;
	while (std::getline(inputfile, line)) {
		if (line.empty())
			continue;
		size_t sep = line.find(":");
		if (sep == string::npos)
			continue;
		string key = line.substr(0, sep);
		string value = line.substr(sep + 1);

		key.erase(0, key.find_first_not_of(" \t"));
		key.erase(key.find_last_not_of(" \t") + 1);
		value.erase(0, value.find_first_not_of(" \t"));
		value.erase(value.find_last_not_of(" \t") + 1);

		cam_params[key] = value;
	}

	inputfile.close();

	int width = std::stoi(cam_params["cam_width"]);
	int height = std::stoi(cam_params["cam_height"]);
	float fx = std::stof(cam_params["cam_fx"]);
	float fy = std::stof(cam_params["cam_fy"]);
	float cx = std::stof(cam_params["cam_cx"]);
	float cy = std::stof(cam_params["cam_cy"]);
	std::string model = "PINHOLE";

	std::ofstream outputfile(output_path);
	if (!outputfile) {
		cout << "写入文件错误" << endl;
		return;
	}

	outputfile << "# Camera list with one line of data per camera:\n";
	outputfile << "#   CAMERA_ID, MODEL, WIDTH, HEIGHT, PARAMS[]\n";
	outputfile << "# Number of cameras: 1\n";
	outputfile << "1 " << model << " " << width << " " << height << " ";
	outputfile << fx << " " << fy << " " << cx << " " << cy << "\n";

	outputfile.close();

	cout << "写入相机文件camera.txt成功" << endl;
}