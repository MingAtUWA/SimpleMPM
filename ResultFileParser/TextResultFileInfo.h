#ifndef _TEXT_RESULT_FILE_INFO_H_
#define _TEXT_RESULT_FILE_INFO_H_

#include <string>

#include "ItemArray.hpp"
#include "ContinuousFlexibleSizeMemory.h"
#include "LinkedFlexibleSizeMemory.h"

/*====================================================
 * Class TextResultFileInfo
 *====================================================
 * Represent general structure of result file as tree:
 *
 * ResultFile
 *  |--TimeHistory1
 *  |   |--Step1
 *  |   |--TimeRecord1
 *  |   |--TimeRecord2
 *  |   |...
 *  |   |--Step2
 *  |   |--TimeRecrod3
 *  |   |--TimeRecord4
 *  |   |...
 *  |--TimeHistory2
 *  |   |--Step2
 *  |   |--TimeRecord1
 *  |   |...
 *  |...
 *  |--ModelState
 *  |   |--BackgroundMesh
 *
 *====================================================*/

struct CommentNode;
struct UnknownNode;
struct StepNode;
struct TimeRecordNode;
struct TimeHistoryNode;
struct BackgroundMeshNode;
struct ObjectNode;
struct ModelStateNode;
class TextResultFileInfo;

struct CommentNode
{
public:
	// position in result file
	size_t begin_pos;
	size_t end_pos;
	size_t begin_line;
	size_t end_line;
	size_t begin_column;
	size_t end_column;

public:
	CommentNode() :
		begin_pos(0), end_pos(0),
		begin_line(0), end_line(0),
		begin_column(0), end_column(0) {}
	~CommentNode() {}
};

struct UnknownNode
{
public:
	// position in result file
	size_t begin_pos;
	size_t end_pos;
	size_t begin_line;
	size_t end_line;
	size_t begin_column;
	size_t end_column;

public:
	UnknownNode(const char *sec_na) :
		section_name(sec_na), 
		begin_pos(0), end_pos(0),
		begin_line(0), end_line(0), 
		begin_column(0), end_column(0) {}
	~UnknownNode() {}
	inline const char *get_section_name(void) const
	{
		return section_name.c_str();
	}

protected:
	const std::string section_name;
};

struct StepNode
{
public:
	std::string name;
	size_t index;
	double start_time;
	double step_time;
	size_t start_substep_index;
	size_t substep_num;

public:
	StepNode() : index(0), start_time(0.0), step_time(0.0),
		start_substep_index(0), substep_num(0) {}
	~StepNode() {}
	void print(std::ostream &out, size_t depth = 0);

protected:
	friend TimeHistoryNode;
	// only used by class Output
	TimeHistoryNode *parent_time_history;
	StepNode *prev_by_time_history;
	StepNode *next_by_time_history;
};

struct FieldDataNode
{
public:
	// field data position in result file
	size_t begin_pos;
	size_t end_pos;
	size_t begin_line;
	size_t end_line;
	size_t begin_column;
	size_t end_column;

public:
	FieldDataNode() : 
		begin_pos(0), end_pos(0), 
		begin_line(0), end_line(0),
		begin_column(0), end_column(0) {}
	~FieldDataNode() {}
};

struct TimeRecordNode
{
public:
	size_t index;
	size_t step_index;
	size_t substep_num;
	size_t total_substep_num;
	double current_time;
	double total_time;
	size_t point_num;
	MemoryUtilities::ItemArray<size_t> point_index;
	FieldDataNode field_data;

public:
	TimeRecordNode() :
		step_index(0), substep_num(0), total_substep_num(0),
		current_time(0.0), total_time(0.0),
		point_num(0), point_index(sizeof(size_t)) {}
	~TimeRecordNode() {}
	void print(std::ostream &out, size_t depth = 0);

protected:
	friend TimeHistoryNode;
	// only used by class Output
	TimeHistoryNode *parent_time_history;
	TimeRecordNode *prev_by_time_history;
	TimeRecordNode *next_by_time_history;
};

struct TimeHistoryNode
{
public:
	std::string name;
	size_t index;
	std::string type;
	size_t interval_num;
	size_t field_num;

public:
	TimeHistoryNode() :
		index(0), interval_num(0), field_num(0),
		head_step(nullptr), tail_step(nullptr),
		head_time_rcd(nullptr), tail_time_rcd(nullptr) {}
	~TimeHistoryNode() { clear(); }
	inline size_t get_field_num(void) const { return field_num; }
	void clear(void)
	{
		StepNode *step_tmp;
		while (tail_step)
		{
			step_tmp = tail_step;
			tail_step = tail_step->prev_by_time_history;
			step_tmp->~StepNode();
		}
		
		TimeRecordNode *time_rcd_tmp;
		while (tail_time_rcd)
		{
			time_rcd_tmp = tail_time_rcd;
			tail_time_rcd = tail_time_rcd->prev_by_time_history;
			time_rcd_tmp->~TimeRecordNode();
		}
	}
	void print(std::ostream &out, size_t depth = 0);

	// ---------------------- step list ----------------------
protected:
	StepNode *head_step;
	StepNode *tail_step;
public:
	StepNode *first_step(void) const { return head_step; }
	StepNode *last_step(void) const { return tail_step; }
	StepNode *next_step(StepNode *cur) const { return cur->next_by_time_history; }
	StepNode *prev_step(StepNode *cur) const { return cur->prev_by_time_history; }
	void add_step(StepNode *stp)
	{
		if (!stp) return;
		stp->parent_time_history = this;
		if (tail_step)
		{
			stp->next_by_time_history = nullptr;
			stp->prev_by_time_history = tail_step;
			tail_step->next_by_time_history = stp;
			tail_step = stp;
		}
		else
		{
			stp->prev_by_time_history = nullptr;
			stp->next_by_time_history = nullptr;
			head_step = stp;
			tail_step = stp;
		}
	}
	
	// ---------------------------- time record list ---------------------------
protected:
	TimeRecordNode *head_time_rcd;
	TimeRecordNode *tail_time_rcd;
public:
	TimeRecordNode *first_time_rcd(void) { return head_time_rcd; }
	TimeRecordNode *last_time_rcd(void) const { return tail_time_rcd; }
	TimeRecordNode *next_time_rcd(TimeRecordNode *cur) { return cur->next_by_time_history; }
	TimeRecordNode *prev_time_rcd(TimeRecordNode *cur) { return cur->prev_by_time_history; }
	void add_time_rcd(TimeRecordNode *trcd)
	{
		if (!trcd) return;
		trcd->parent_time_history = this;
		if (tail_time_rcd)
		{
			trcd->next_by_time_history = nullptr;
			trcd->prev_by_time_history = tail_time_rcd;
			tail_time_rcd->next_by_time_history = trcd;
			tail_time_rcd = trcd;
		}
		else
		{
			trcd->prev_by_time_history = nullptr;
			trcd->next_by_time_history = nullptr;
			head_time_rcd = trcd;
			tail_time_rcd = trcd;
		}
	}

protected:
		friend TextResultFileInfo;
		// only used by class ResultFile
		TextResultFileInfo *parent_result_file;
		TimeHistoryNode *prev_by_resultfile, *next_by_resultfile;
};

struct BackgroundMeshNode
{
public:
	std::string type;
	size_t x_coord_num;
	size_t y_coord_num;
	MemoryUtilities::ItemArray<double> x_coords;
	MemoryUtilities::ItemArray<double> y_coords;

public:
	BackgroundMeshNode() : x_coord_num(0), y_coord_num(0) {}
	~BackgroundMeshNode() {}

	void print(std::ostream &out, size_t depth = 0);
};

struct ObjectNode
{
public:
	std::string type;
	size_t particle_num;
	MemoryUtilities::ItemArray<double> x;
	MemoryUtilities::ItemArray<double> y;
	MemoryUtilities::ItemArray<double> vol;
	MemoryUtilities::ItemArray<double> n;
	MemoryUtilities::ItemArray<double> density_s;
	MemoryUtilities::ItemArray<double> density_f;
	MemoryUtilities::ItemArray<double> density;

public:
	ObjectNode() : particle_num(0),
		parent_model_state(nullptr),
		prev_by_model_state(nullptr),
		next_by_model_state(nullptr) {}
	~ObjectNode() {}

	void print(std::ostream &out, size_t depth = 0);

protected:
	friend ModelStateNode;
	ModelStateNode *parent_model_state;
	ObjectNode *prev_by_model_state, *next_by_model_state;
};


struct ModelStateNode
{
public:
	std::string name;
	size_t index;
	std::string type;
	
public:
	ModelStateNode() : index(0),
		head_object(nullptr), tail_object(nullptr),
		background_mesh(nullptr) {}
	~ModelStateNode() {}
	void print(std::ostream &out, size_t depth = 0);

	//-------------------- Object_list ----------------------
protected:
	ObjectNode *head_object;
	ObjectNode *tail_object;
public:
	ObjectNode *first_object(void) { return head_object; }
	ObjectNode *last_object(void) const { return tail_object; }
	ObjectNode *next_object(ObjectNode *cur) { return cur->next_by_model_state; }
	ObjectNode *prev_object(ObjectNode *cur) { return cur->prev_by_model_state; }
	void add_object(ObjectNode *obj)
	{
		if (!obj) return;
		obj->parent_model_state = this;
		if (tail_object)
		{
			obj->next_by_model_state = nullptr;
			obj->prev_by_model_state = tail_object;
			tail_object->next_by_model_state = obj;
			tail_object = obj;
		}
		else
		{
			obj->prev_by_model_state = nullptr;
			obj->next_by_model_state = nullptr;
			head_object = obj;
			tail_object = obj;
		}
	}

	BackgroundMeshNode *background_mesh;

protected:
	friend TextResultFileInfo;
	// only used by class ResultFile
	TextResultFileInfo *parent_result_file;
	ModelStateNode *prev_by_resultfile, *next_by_resultfile;
};

class TextResultFileInfo
{
public:
	std::string name;

public:
	TextResultFileInfo() : 
		head_time_history(nullptr), tail_time_history(nullptr),
		head_model_state(nullptr), tail_model_state(nullptr),
		memory(4 * 1024) {}
	~TextResultFileInfo() { clear(); }
	void clear(void)
	{
		// Clear time histories
		TimeHistoryNode *th_tmp;
		while (tail_time_history)
		{
			th_tmp = tail_time_history;
			tail_time_history = tail_time_history->prev_by_resultfile;
			th_tmp->~TimeHistoryNode();
		}
		head_time_history = nullptr;
		// Clear model states 
		ModelStateNode *ms_tmp;
		while (tail_model_state)
		{
			ms_tmp = tail_model_state;
			tail_model_state = tail_model_state->prev_by_resultfile;
			ms_tmp->~ModelStateNode();
		}
		head_model_state = nullptr;
		// Clear memory
		memory.clear();
	}
	void print(std::ostream &out, size_t depth = 0);

	// ---------------------------- Output list -----------------------------
protected:
	TimeHistoryNode *head_time_history;
	TimeHistoryNode *tail_time_history;
public:
	TimeHistoryNode *first_time_history(void) const { return head_time_history; }
	TimeHistoryNode *last_time_history(void) const { return tail_time_history; }
	TimeHistoryNode *next_time_history(TimeHistoryNode *cur) const { return cur->next_by_resultfile; }
	TimeHistoryNode *prev_time_history(TimeHistoryNode *cur) const { return cur->prev_by_resultfile; }
	void add_time_history(TimeHistoryNode *out)
	{
		if (!out) return;
		out->parent_result_file = this;
		if (tail_time_history)
		{
			out->next_by_resultfile = nullptr;
			out->prev_by_resultfile = tail_time_history;
			tail_time_history->next_by_resultfile = out;
			tail_time_history = out;
		}
		else
		{
			out->prev_by_resultfile = nullptr;
			out->next_by_resultfile = nullptr;
			head_time_history = out;
			tail_time_history = out;
		}
	}

	// ---------------------------- ModelState list -----------------------------
protected:
	ModelStateNode *head_model_state;
	ModelStateNode *tail_model_state;
public:
	ModelStateNode *first_model_state(void) const { return head_model_state; }
	ModelStateNode *last_model_state(void) const { return tail_model_state; }
	ModelStateNode *next_model_state(ModelStateNode *cur) const { return cur->next_by_resultfile; }
	ModelStateNode *prev_model_state(ModelStateNode *cur) const { return cur->prev_by_resultfile; }
	void add_model_state(ModelStateNode *ms)
	{
		if (!ms) return;
		ms->parent_result_file = this;
		if (tail_model_state)
		{
			ms->next_by_resultfile = nullptr;
			ms->prev_by_resultfile = tail_model_state;
			tail_model_state->next_by_resultfile = ms;
			tail_model_state = ms;
		}
		else
		{
			ms->prev_by_resultfile = nullptr;
			ms->next_by_resultfile = nullptr;
			head_model_state = ms;
			tail_model_state = ms;
		}
	}

protected:
	LinkedFlexibleSizeMemory memory;
public:
	CommentNode *new_comment(void)
	{
		return new (memory.alloc(sizeof(CommentNode))) CommentNode;
	}
	UnknownNode *new_unknown(const char *section_name)
	{
		return new (memory.alloc(sizeof(UnknownNode))) UnknownNode(section_name);
	}
	StepNode *new_step(void)
	{
		return new (memory.alloc(sizeof(StepNode))) StepNode;
	}
	TimeRecordNode *new_time_rcd(void)
	{
		return new (memory.alloc(sizeof(TimeRecordNode))) TimeRecordNode;
	}
	TimeHistoryNode *new_time_history(void)
	{
		return new (memory.alloc(sizeof(TimeHistoryNode))) TimeHistoryNode;
	}
	BackgroundMeshNode *new_background_mesh(void)
	{
		return new (memory.alloc(sizeof(BackgroundMeshNode))) BackgroundMeshNode;
	}
	ObjectNode *new_object(void)
	{
		return new (memory.alloc(sizeof(ObjectNode))) ObjectNode;
	}
	ModelStateNode *new_model_state(void)
	{
		return new (memory.alloc(sizeof(ModelStateNode))) ModelStateNode;
	}
};

#endif