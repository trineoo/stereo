#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/ximgproc.hpp"
#include <gtk/gtk.h>
#include <cstdio>
#include <cstring>
#include <ctime>


using namespace std;
using namespace cv;
using namespace cv::ximgproc;

/* Matcher type */
typedef enum {
	BM
} MatcherType;

/* Main data structure definition */
struct ChData {
	/* Widgets */
	GtkWidget *main_window; /* Main application window */
	GtkImage *image_left;
	GtkImage *image_right;
	GtkImage *image_depth;
	GtkWidget *rb_bm;
	GtkWidget *sc_block_size, *sc_min_disparity, *sc_num_disparities, *sc_lambda, *sc_sigma;
	GtkAdjustment *adj_block_size, *adj_min_disparity, *adj_num_disparities, *adj_lambda, *adj_sigma;
	GtkWidget *status_bar;
	gint status_bar_context;

	/* OpenCV */
	Ptr<StereoMatcher> stereo_matcher;
	Mat cv_left_for_matcher, cv_right_for_matcher,
	 		cv_left_disp, cv_right_disp,
			cv_filtered_disp, cv_filtered_disp_vis;
	Ptr<StereoMatcher> right_matcher;
	Ptr<DisparityWLSFilter> wls_filter;

	Mat cv_image_left, cv_image_right, cv_image_disparity,
			cv_image_disparity_normalized, cv_color_image;
	MatcherType matcher_type;
	int block_size;
	int min_disparity;
	int num_disparities;
	int lambda;
	float sigma;
	bool live_update;

	/* Defalt values */
	static const int DEFAULT_BLOCK_SIZE = 9;
	static const int DEFAULT_NUM_DISPARITIES = 144;
	static const int DEFAULT_LAMBDA = 8000;
	static const float DEFAULT_SIGMA = 1.5;
	//static const int DEFAULT_MIN_DISPARITY = 0;

	ChData() : matcher_type(BM), block_size(DEFAULT_BLOCK_SIZE), //min_disparity(DEFAULT_MIN_DISPARITY),
			num_disparities(DEFAULT_NUM_DISPARITIES), lambda(DEFAULT_LAMBDA), sigma(DEFAULT_SIGMA),
			live_update(true)
		{}
};

void update_matcher(ChData *data) {
	if(!data->live_update) {
		return;
	}

	Ptr<StereoBM> left_matcher;
 	left_matcher = data->stereo_matcher.dynamicCast<StereoBM>();
	Ptr<DisparityWLSFilter> wls_filter;
	wls_filter = data->wls_filter.dynamicCast<DisparityWLSFilter>();
	Ptr<StereoMatcher> right_matcher;
	right_matcher = data->right_matcher.dynamicCast<StereoMatcher>();


	//If we have the wrong type of matcher, let's create a new one:
	if (!left_matcher) {
			data->cv_left_for_matcher = (data->cv_image_left).clone();
			data->cv_right_for_matcher = (data->cv_image_right).clone();
			data->stereo_matcher = left_matcher = StereoBM::create(data->num_disparities, data->block_size);
			data->wls_filter = wls_filter = createDisparityWLSFilter(left_matcher);
			data->right_matcher = right_matcher = createRightMatcher(left_matcher);
			cvtColor(data->cv_left_for_matcher,  data->cv_left_for_matcher,  COLOR_BGR2GRAY);
			cvtColor(data->cv_right_for_matcher, data->cv_right_for_matcher, COLOR_BGR2GRAY);
			gtk_widget_set_sensitive(data->sc_block_size, true);
			gtk_widget_set_sensitive(data->sc_num_disparities, true);
			gtk_widget_set_sensitive(data->sc_lambda, true);
			gtk_widget_set_sensitive(data->sc_sigma, true);
			//gtk_widget_set_sensitive(data->sc_min_disparity, true);
	}
	data->stereo_matcher->setBlockSize(data->block_size);
	data->stereo_matcher->setNumDisparities(data->num_disparities);
	data->wls_filter->setLambda(data->lambda);
	data->wls_filter->setSigmaColor(data->sigma);
	//stereo_bm->setMinDisparity(data->min_disparity);


	clock_t t;
	t = clock();
	data->stereo_matcher->compute(data->cv_left_for_matcher, data->cv_right_for_matcher, data->cv_left_disp);
	data->right_matcher->compute(data->cv_right_for_matcher, data->cv_left_for_matcher, data->cv_right_disp);
	data->wls_filter->filter(data->cv_left_disp,data->cv_image_left, data->cv_image_disparity, data->cv_right_disp);
	t = clock() - t;

	gchar *status_message = g_strdup_printf("Disparity computation took %lf milliseconds",((double)t*1000)/CLOCKS_PER_SEC);
	gtk_statusbar_pop(GTK_STATUSBAR(data->status_bar), data->status_bar_context);
	gtk_statusbar_push(GTK_STATUSBAR(data->status_bar), data->status_bar_context, status_message);
	g_free(status_message);

	//imwrite("rawDisparitymap.bmp",data->cv_image_disparity);
	getDisparityVis(data->cv_image_disparity,data->cv_filtered_disp_vis,1.0);
	//If the value is out of range, it is clamped to the minimum or maximum values.
	//imwrite("clampedDisparity.bmp",data->cv_filtered_disp_vis);
	normalize(data->cv_filtered_disp_vis, data->cv_filtered_disp_vis, 0, 255, NORM_MINMAX);
	bitwise_not( data->cv_filtered_disp_vis, data->cv_filtered_disp_vis);
  applyColorMap(data->cv_filtered_disp_vis, data->cv_color_image, COLORMAP_JET);
	//imwrite("applyColorMap.bmp",data->cv_color_image);
	Mat dummy_to_scale= (data->cv_color_image).clone();
	resize(dummy_to_scale, dummy_to_scale, Size(), 0.58, 0.58, CV_INTER_AREA);
	GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
			(guchar*) dummy_to_scale.data, GDK_COLORSPACE_RGB, false,
			8, dummy_to_scale.cols,
			dummy_to_scale.rows, dummy_to_scale.step,
			NULL, NULL);
	gtk_image_set_from_pixbuf(data->image_depth, pixbuf);
}

void update_interface(ChData *data) {
	//Avoids rebuilding the matcher on every change:
	data->live_update = false;

	if(data->matcher_type == BM) {
		gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(data->rb_bm),true);
	}

	gtk_adjustment_set_value(data->adj_block_size,data->block_size);
	gtk_adjustment_set_value(data->adj_num_disparities,data->num_disparities);
	gtk_adjustment_set_value(data->adj_lambda,data->lambda);
	gtk_adjustment_set_value(data->adj_sigma,data->sigma);
	//gtk_adjustment_set_value(data->adj_min_disparity,data->min_disparity);

	data->live_update = true;
	update_matcher(data);
}

extern "C" {
G_MODULE_EXPORT void on_adj_block_size_value_changed(GtkAdjustment *adjustment,
		ChData *data) {
	gint value;

	if (data == NULL) {
		fprintf(stderr, "WARNING: data is null\n");
		return;
	}

	value = (gint) gtk_adjustment_get_value(adjustment);

	//the value must be odd, if it is not then set it to the next odd value
	if (value % 2 == 0) {
		value += 1;
		gtk_adjustment_set_value(adjustment, (gdouble) value);
		return;
	}

	//the value must be smaller than the image size
	if (value >= data->cv_image_left.cols
			|| value >= data->cv_image_left.rows) {
		fprintf(stderr, "WARNING: Block size is larger than image size\n");
		return;
	}

	//set the parameter,
	data->block_size = value;
	update_matcher(data);
}

G_MODULE_EXPORT void on_adj_min_disparity_value_changed(
		GtkAdjustment *adjustment, ChData *data) {
	gint value;

	if (data == NULL) {
		fprintf(stderr, "WARNING: data is null\n");
		return;
	}

	value = (gint) gtk_adjustment_get_value(adjustment);

	data->min_disparity = value;
	update_matcher(data);
}

G_MODULE_EXPORT void on_adj_num_disparities_value_changed( GtkAdjustment *adjustment, ChData *data ) {
	gint value;

	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}

	value = (gint) gtk_adjustment_get_value( adjustment );

	//The value must be divisible by 16, if it is not set it to the nearest multiple of 16
	if (value % 16 != 0)
	{
		value += (16 - value%16);
		gtk_adjustment_set_value( adjustment, (gdouble)value);
		return;
	}

	data->num_disparities = value;
	update_matcher(data);
}

G_MODULE_EXPORT void on_adj_lambda_value_changed( GtkAdjustment *adjustment, ChData *data ) {
	gint value;

	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}

	value = (gint) gtk_adjustment_get_value( adjustment );

	data->lambda = value;
	update_matcher(data);
}

G_MODULE_EXPORT void on_adj_sigma_value_changed( GtkAdjustment *adjustment, ChData *data ) {
	gfloat value;

	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}

	value = (gfloat) gtk_adjustment_get_value( adjustment );

	data->sigma = value;
	update_matcher(data);
}


G_MODULE_EXPORT void on_algo_sbm_clicked(GtkButton *b, ChData *data) {
	if(gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(b))) {
		data->matcher_type = BM;
		update_matcher(data);
	}
}


G_MODULE_EXPORT void on_btn_save_clicked(GtkButton *b, ChData *data) {
	GtkWidget *dialog;
	GtkFileChooser *chooser;
	GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_SAVE;
	gint res;

	dialog = gtk_file_chooser_dialog_new("Save File", GTK_WINDOW(data->main_window), action, "Cancel", GTK_RESPONSE_CANCEL, "Save", GTK_RESPONSE_ACCEPT, NULL);
	chooser = GTK_FILE_CHOOSER(dialog);
	gtk_file_chooser_set_do_overwrite_confirmation(chooser, TRUE);
	gtk_file_chooser_set_current_name(chooser, "params.yml");

	GtkFileFilter *filter_yml = gtk_file_filter_new();
	gtk_file_filter_set_name(filter_yml,"YAML file (*.yml)");
	gtk_file_filter_add_pattern(filter_yml,"*.yml");

	GtkFileFilter *filter_xml = gtk_file_filter_new();
	gtk_file_filter_set_name(filter_xml, "XML file(*.xml)");
	gtk_file_filter_add_pattern(filter_xml,"*.xml");

	gtk_file_chooser_add_filter(chooser,filter_yml);
	gtk_file_chooser_add_filter(chooser,filter_xml);

	res = gtk_dialog_run(GTK_DIALOG(dialog));
	char *filename;
	filename = gtk_file_chooser_get_filename(chooser);
	gtk_widget_destroy(GTK_WIDGET(dialog));

	if(res == GTK_RESPONSE_ACCEPT) {
		int len = strlen(filename);

		if(!strcmp(filename+len-4,".yml") || !strcmp(filename+len-4,".xml")) {
			FileStorage fs(filename, FileStorage::WRITE);

			fs <<
			"name" << "StereoMatcher.BM" <<
			"blockSize" << data->block_size <<
			"minDisparity" << data->min_disparity <<
			"numDisparities" << data->num_disparities <<
			"lambda" << data->lambda <<
			"sigma" << data->sigma;
			fs.release();

			GtkWidget *message = gtk_message_dialog_new(GTK_WINDOW(data->main_window), GTK_DIALOG_DESTROY_WITH_PARENT, GTK_MESSAGE_INFO, GTK_BUTTONS_CLOSE, "Parameters saved successfully");
			gtk_dialog_run(GTK_DIALOG(message));
			gtk_widget_destroy(GTK_WIDGET(message));
		} else {
			GtkWidget *message = gtk_message_dialog_new(GTK_WINDOW(data->main_window), GTK_DIALOG_DESTROY_WITH_PARENT, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "Currently the only supported formats are XML and YAML.");
			gtk_dialog_run(GTK_DIALOG(message));
			gtk_widget_destroy(GTK_WIDGET(message));
		}

		g_free(filename);
	}
}

G_MODULE_EXPORT void on_btn_load_clicked(GtkButton *b, ChData *data) {
	GtkWidget *dialog;
	GtkFileChooser *chooser;
	GtkFileChooserAction action = GTK_FILE_CHOOSER_ACTION_OPEN;
	gint res;

	dialog = gtk_file_chooser_dialog_new("Open File", GTK_WINDOW(data->main_window), action, "Cancel", GTK_RESPONSE_CANCEL, "Open", GTK_RESPONSE_ACCEPT, NULL);
	chooser = GTK_FILE_CHOOSER(dialog);

	GtkFileFilter *filter = gtk_file_filter_new();
	gtk_file_filter_set_name(filter,"YAML or XML file (*.yml, *.xml)");
	gtk_file_filter_add_pattern(filter,"*.yml");
	gtk_file_filter_add_pattern(filter,"*.xml");

	gtk_file_chooser_add_filter(chooser,filter);

	res = gtk_dialog_run(GTK_DIALOG(dialog));
	char *filename;
	filename = gtk_file_chooser_get_filename(chooser);
	gtk_widget_destroy(GTK_WIDGET(dialog));

	if(res == GTK_RESPONSE_ACCEPT) {
		int len = strlen(filename);

		if(!strcmp(filename+len-4,".yml") || !strcmp(filename+len-4,".xml")) {
			FileStorage fs(filename, FileStorage::READ);

			if(!fs.isOpened()) {
				GtkWidget *message = gtk_message_dialog_new(GTK_WINDOW(data->main_window), GTK_DIALOG_DESTROY_WITH_PARENT, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "Could not open the selected file.");
				gtk_dialog_run(GTK_DIALOG(message));
				gtk_widget_destroy(GTK_WIDGET(message));
			} else {
				string name;
				fs["name"] >> name;

				if(name == "StereoMatcher.BM") {
					data->matcher_type = BM;
					fs["blockSize"] >> data->block_size;
					fs["minDisparity"] >> data->min_disparity;
					fs["numDisparities"] >> data->num_disparities;
					fs["lambda"] >> data->lambda;
					fs["sigma"] >> data->sigma;
					update_interface(data);

					GtkWidget *message = gtk_message_dialog_new(GTK_WINDOW(data->main_window), GTK_DIALOG_DESTROY_WITH_PARENT, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "Parameters loaded successfully.");
					gtk_dialog_run(GTK_DIALOG(message));
					gtk_widget_destroy(GTK_WIDGET(message));
				} else {
					GtkWidget *message = gtk_message_dialog_new(GTK_WINDOW(data->main_window), GTK_DIALOG_DESTROY_WITH_PARENT, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "This file is not valid.");
					gtk_dialog_run(GTK_DIALOG(message));
					gtk_widget_destroy(GTK_WIDGET(message));
				}

				fs.release();
			}
		} else {
			GtkWidget *message = gtk_message_dialog_new(GTK_WINDOW(data->main_window), GTK_DIALOG_DESTROY_WITH_PARENT, GTK_MESSAGE_ERROR, GTK_BUTTONS_CLOSE, "Currently the only supported formats are XML and YAML.");
			gtk_dialog_run(GTK_DIALOG(message));
			gtk_widget_destroy(GTK_WIDGET(message));
		}

		g_free(filename);
	}
}
G_MODULE_EXPORT void on_btn_defaults_clicked(GtkButton *b, ChData *data) {
	data->matcher_type = BM;
	data->block_size = ChData::DEFAULT_BLOCK_SIZE;
	//data->min_disparity = ChData::DEFAULT_MIN_DISPARITY;
	data->num_disparities = ChData::DEFAULT_NUM_DISPARITIES;
	data->lambda = ChData::DEFAULT_LAMBDA;
	data->sigma = ChData::DEFAULT_SIGMA;
	update_interface(data);
}
}

int main(int argc, char *argv[]) {

	char default_left_filename[] = "../leftRect3.bmp";
	char default_right_filename[] = "../rightRect3.bmp";
	char *left_filename = default_left_filename;
	char *right_filename = default_right_filename;

	GtkBuilder *builder;
	GError *error = NULL;
	ChData *data;

	//Read images to tune
	Mat left_image = imread(left_filename,IMREAD_COLOR);
	if(left_image.empty()) {printf("Could not read left image %s.\n",left_filename);exit(1);}
	Mat right_image = imread(right_filename,IMREAD_COLOR);
	if(right_image.empty()) {printf("Could not read right image %s.\n",right_filename);exit(1);}
	if(left_image.size() != right_image.size()) {printf("Left and right images have different sizes.\n");exit(1);}

	/* Create data */
	data = new ChData();

	data->cv_image_left = 	(left_image).clone();
	data->cv_image_right = 	(right_image).clone();;

	/* Init GTK+ */
	gtk_init(&argc, &argv);

	/* Create new GtkBuilder object */
	builder = gtk_builder_new();

	if (!gtk_builder_add_from_file(builder, "../StereoTuner.glade", &error)) {
		g_warning("%s", error->message);
		g_free(error);
		return (1);
	}

	/* Get main window pointer from UI */
	data->main_window = GTK_WIDGET(gtk_builder_get_object(builder, "window1"));
	data->image_left = GTK_IMAGE(gtk_builder_get_object(builder, "image_left"));
	data->image_right = GTK_IMAGE(gtk_builder_get_object(builder, "image_right"));
	data->image_depth = GTK_IMAGE(gtk_builder_get_object(builder, "image_disparity"));

	data->sc_block_size = GTK_WIDGET(gtk_builder_get_object(builder, "sc_block_size"));
	data->sc_min_disparity = GTK_WIDGET(gtk_builder_get_object(builder, "sc_min_disparity"));
	data->sc_num_disparities = GTK_WIDGET(gtk_builder_get_object(builder, "sc_num_disparities"));
	data->sc_lambda = GTK_WIDGET(gtk_builder_get_object(builder, "sc_lambda"));
	data->sc_sigma = GTK_WIDGET(gtk_builder_get_object(builder, "sc_sigma"));
	data->status_bar = GTK_WIDGET(gtk_builder_get_object(builder, "status_bar"));
	data->rb_bm = GTK_WIDGET(gtk_builder_get_object(builder, "algo_sbm"));
	data->adj_block_size = GTK_ADJUSTMENT(gtk_builder_get_object(builder, "adj_block_size"));
	data->adj_min_disparity = GTK_ADJUSTMENT(gtk_builder_get_object(builder, "adj_min_disparity"));
	data->adj_num_disparities = GTK_ADJUSTMENT(gtk_builder_get_object(builder, "adj_num_disparities"));
	data->adj_lambda = GTK_ADJUSTMENT(gtk_builder_get_object(builder, "adj_lambda"));
	data->adj_sigma = GTK_ADJUSTMENT(gtk_builder_get_object(builder, "adj_sigma"));
	data->status_bar_context = gtk_statusbar_get_context_id(GTK_STATUSBAR(data->status_bar), "Statusbar context");


	Mat leftRGB, rightRGB;
	cvtColor(left_image, leftRGB,
			CV_BGR2RGB);
	resize(leftRGB, leftRGB, Size(), 0.35, 0.35, CV_INTER_AREA);
	GdkPixbuf *pixbuf = gdk_pixbuf_new_from_data(
			(guchar*) leftRGB.data, GDK_COLORSPACE_RGB, false,
			8, leftRGB.cols,
			leftRGB.rows, leftRGB.step,
			NULL, NULL);
	gtk_image_set_from_pixbuf(data->image_left, pixbuf);

	cvtColor(right_image, rightRGB,
			CV_BGR2RGB);
	resize(rightRGB, rightRGB, Size(), 0.35, 0.35, CV_INTER_AREA);
	pixbuf = gdk_pixbuf_new_from_data(
			(guchar*) rightRGB.data, GDK_COLORSPACE_RGB, false,
			8, rightRGB.cols,
			rightRGB.rows, rightRGB.step,
			NULL, NULL);
	gtk_image_set_from_pixbuf(data->image_right, pixbuf);

	update_matcher(data);

	/* Connect signals */
	gtk_builder_connect_signals(builder, data);

	/* Destroy builder, since we don't need it anymore */
	g_object_unref(G_OBJECT(builder));

	/* Show window. All other widgets are automatically shown by GtkBuilder */
	gtk_widget_show(data->main_window);

	/* Start main loop */
	gtk_main();

	return (0);
}
