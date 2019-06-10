#pragma once
class VSStabHandle
{
public:
	VSVideo *video = 0;
	bool videoOK = false;

	VSFeatureModel *featureModel = 0;
	VSFeatureModel::VSFeatureModelType featureModelType;
	bool featureModelOK = false;

	VSStabHandle();
	~VSStabHandle();

	void OpenAVideo(char *fileName); //Open a video File;
	void MakeFeatureModel();
	void GetFeatures()
};

