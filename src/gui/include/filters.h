/* 
	filters.h
	Header file for filters for vision task
	Date created: 8 Jan 2014
	Author: Thein
*/

#ifndef FILTERS_H_
#define FILTERS_H_

#include <vector>
#include <string>

//Abstract base class for filters
class Filter {
protected:
	cv::Mat inImage;
public:
    Filter() {}
	Filter(cv::Mat image) {
		this->inImage = image;
	}
    virtual ~Filter() {}
	virtual void setInputImage(cv::Mat);
	virtual cv::Mat getOutputImage() = 0;
	virtual std::string getName() = 0;
};

class CannyFilter : public Filter {
private:
	static std::string name;
public:
    CannyFilter() {}
    CannyFilter(cv::Mat image) : Filter(image) {}
	cv::Mat getOutputImage();
    std::string getName() { return CannyFilter::name; }
};

class AdaptiveThresholdFilter: public Filter {
private:
	static std::string name;
public:
    AdaptiveThresholdFilter() {}
    AdaptiveThresholdFilter(cv::Mat image) : Filter(image) {}
	cv::Mat getOutputImage();
	std::string getName() { return AdaptiveThresholdFilter::name; }
};

class BlackLineCenter : public Filter {
private:
	static std::string name;
	double thVal;
	double areaThresh;
public:
	BlackLineCenter();
	//BlackLineCenter(cv::Mat image);
	cv::Mat getOutputImage();
	std::string getName() { return BlackLineCenter::name; }
};

class FiltersContainer {
private:
	Filter *filter1, *filter2, *filter3;
	std::vector<Filter*> front_filters, bottom_filters;
	static const int NUM_FRONT_FILTERS, NUM_BOTTOM_FILTERS;
public:
	typedef std::vector<Filter*> Filters;

	FiltersContainer();
	std::vector<Filter*> getFrontFilters();
	std::vector<Filter*> getBottomFilters();
	~FiltersContainer();
};

#endif /* FILTERS_H_ */
