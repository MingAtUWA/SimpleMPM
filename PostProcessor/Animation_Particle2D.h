#ifndef _ANIMATION_PARTICLE2D_H_
#define _ANIMATION_PARTICLE2D_H_

#include "ItemArray.hpp"
#include "FileCharArray.h"
#include "TextResultFileParser.hpp"
#include "FieldDataParser.hpp"
#include "Frame_Particle2D.h"

#include "Animation.h"

class Animation_Particle2D : public Animation
{
public:
	Animation_Particle2D(double animation_time, vtkRenderWindowInteractor *win_iter);
	~Animation_Particle2D();

	int init(const char *res_file_name);
	int get_background_mesh(const char *mesh_file_name);

	double get_frame_time(size_t frame_index) override;
	vtkRenderer *render_frame(size_t frame_index) override;

protected:
	FileCharArray file;
	TextResultFile::Parser<PFileCharArray> parser;
	TextResultFile::FieldDataParser<PFileCharArray> fld_parser;
	TextResultFileInfo res_file_info;
	TimeHistoryNode *time_history;
	
	MemoryUtilities::ItemArray<TimeRecordNode *> time_rcds;
	// pcl x, y coordinates and volume
	MemoryUtilities::ItemArray<double> pcl_x_coord;
	MemoryUtilities::ItemArray<double> pcl_y_coord;
	MemoryUtilities::ItemArray<double> pcl_vol;
	MemoryUtilities::ItemArray<double> pcl_fld;
	// render
	Frame_Particle2D frame;
};

#endif