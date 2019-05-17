#include "TextResultFileInfo.h"

// all the print function need to output as debug function in the future
#define SPACE_PADDING_NUM 2

void StepNode::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');
	out << pad_space << "Step" << "\n"
		<< pad_space << "  name: " << name << "\n"
		<< pad_space << "  index: " << index << "\n"
		<< pad_space << "  start time: " << start_time << "\n"
		<< pad_space << "  step time: " << step_time << "\n"
		<< pad_space << "  start substep index: " << start_substep_index << "\n"
		<< pad_space << "  substep number: " << substep_num << std::endl;
}

void TimeRecordNode::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');
	out << pad_space << "TimeRecord" << "\n"
		<< pad_space << "  index: " << index << "\n"
		<< pad_space << "  step index: " << step_index << "\n"
		<< pad_space << "  substep num: " << substep_num << "\n"
		<< pad_space << "  total substep num: " << total_substep_num << "\n"
		<< pad_space << "  current time: " << current_time << "\n"
		<< pad_space << "  total time: " << total_time << "\n"
		<< pad_space << "  point num: " << point_num << "\n";
	// output point index
	out << pad_space << "  point index: ";
	size_t p_num_tmp = point_index.get_num();
	for (size_t i = 0; i < p_num_tmp; i++)
	{
		out << point_index[i] << " ";
	}
	out << "\n";
	// output field data position
	out << pad_space
		<< "  field data: from " << field_data.begin_pos
		<< " (line " << field_data.begin_line
		<< ", col " << field_data.begin_column << ") "
		<< " to " << field_data.end_pos
		<< " (line " << field_data.end_line
		<< ", col " << field_data.end_column << ") " << std::endl;
}

void TimeHistoryNode::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');

	// print itself
	out << pad_space << "Output\n"
		<< pad_space << "  name: " << name << "\n"
		<< pad_space << "  index: " << index << "\n"
		<< pad_space << "  type: " << type << "\n"
		<< pad_space << "  interval num: " << interval_num << "\n"
		<< pad_space << "  field num: " << field_num << std::endl;

	++depth;
	// print step
	for (StepNode *stp_iter = first_step(); stp_iter; stp_iter = next_step(stp_iter))
	{
		stp_iter->print(out, depth);
	}
	// print time record
	for (TimeRecordNode *trcd_iter = first_time_rcd();
		trcd_iter; trcd_iter = next_time_rcd(trcd_iter))
	{
		trcd_iter->print(out, depth);
	}
}

void BackgroundMeshNode::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');

	out << pad_space << "BackgroundMesh\n"
		<< pad_space << "  Type = " << type << "\n"
		<< pad_space << "  XCoordinateNum = " << x_coord_num << "\n"
		<< pad_space << "  YCoordinateNum = " << y_coord_num << "\n";
	if (x_coord_num)
	{
		out << pad_space << "  XCoordinates = ";
		for (size_t i = 0; i < x_coord_num - 1; i++)
			out << x_coords[i] << ", ";
		out << x_coords[x_coord_num - 1] << "\n";
	}
	if (y_coord_num)
	{
		out << pad_space << "  YCoordinates = ";
		for (size_t i = 0; i < y_coord_num - 1; i++)
			out << y_coords[i] << ", ";
		out << y_coords[y_coord_num - 1] << "\n";
	}

}

void ObjectNode::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');

	out << pad_space << "Object\n"
		<< pad_space << "  Type = " << type << "\n"
		<< pad_space << "  ParticleNum = " << particle_num << "\n";
#define DISPLAY_FIELD(field)                           \
	if (field.get_num())                               \
	{                                                  \
		out << pad_space << "  " #field " = ";         \
		for (size_t i = 0; i < field.get_num(); i++)   \
			out << field[i] << " ";                    \
		out << "\n";                                   \
	}
	DISPLAY_FIELD(x);
	DISPLAY_FIELD(y);
	DISPLAY_FIELD(vol);
	DISPLAY_FIELD(n);
	DISPLAY_FIELD(density_s);
	DISPLAY_FIELD(density_f);
#undef DISPLAY_FIELD
}

void ModelStateNode::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');

	// print itself
	out << pad_space << "ModelState\n"
		<< pad_space << "  type: " << type << "\n";

	++depth;
	// background mesh
	if (background_mesh)
		background_mesh->print(out, depth);
	// object
	for (ObjectNode *obj_iter = first_object(); obj_iter;
		 obj_iter = next_object(obj_iter))
		obj_iter->print(out, depth);
}

void TextResultFileInfo::print(std::ostream &out, size_t depth)
{
	std::string pad_space(depth * SPACE_PADDING_NUM, ' ');

	// print itself
	out << pad_space << "Result File " << name << std::endl;

	depth += 1;
	// print output
	for (TimeHistoryNode *th_iter = first_time_history();
		th_iter; th_iter = next_time_history(th_iter))
	{
		th_iter->print(out, depth);
	}

	for (ModelStateNode *ms_iter = first_model_state();
		ms_iter; ms_iter = next_model_state(ms_iter))
	{
		ms_iter->print(out, depth);
	}
}
