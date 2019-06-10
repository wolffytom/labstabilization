#include "stdafx.h"
#include "VSStabHandle.h"

VSStabHandle::VSStabHandle()
{
}

VSStabHandle::~VSStabHandle()
{
}

void VSStabHandle::OpenAVideo(char *fileName)
{
	video = new VSVideo();
	video->Capture.open(fileName); //for Check is the file exist
	std::strcpy(video->FileName, fileName);
	video->Fps = video->Capture.get(CV_CAP_PROP_FPS);
	video->Width = video->Capture.get(CV_CAP_PROP_FRAME_WIDTH);
	video->Height = video->Capture.get(CV_CAP_PROP_FRAME_HEIGHT);
	video->FramesCount = video->Capture.get(CV_CAP_PROP_FRAME_COUNT);

	videoOK = true;
}

void VSStabHandle::MakeFeatureModel()
{
	featureModelType = VSFeatureModel::VSFeatureModelType::AdjacentPointsFeatureModel;
	featureModel = new VSAdjacentPointsFeatureModel(video->FramesCount);
    VSGetFeaturesTool *getFeaturesTool = new VSGetFeaturesTool(video,featureModel,featureModelType);
	featureModelOK = true;
}