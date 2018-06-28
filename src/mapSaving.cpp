#include "mapSaving.h"

void savemap(PLSLAM::MapHandler* map, string path)
{	
	ofstream out(path);
	for(auto ele:map->map_keyframes)	//KeyFrame*
	{
		out<<"f_idx "<<ele->f_idx<<" kf_idx "<<ele->kf_idx<<" ";
		out<<ele->T_kf_w(0,0)<<" "<<ele->T_kf_w(0,1)<<" "<<ele->T_kf_w(0,2)<<" "<<ele->T_kf_w(0,3)<<" ";
		out<<ele->T_kf_w(1,0)<<" "<<ele->T_kf_w(1,1)<<" "<<ele->T_kf_w(1,2)<<" "<<ele->T_kf_w(1,3)<<" ";
		out<<ele->T_kf_w(2,0)<<" "<<ele->T_kf_w(2,1)<<" "<<ele->T_kf_w(2,2)<<" "<<ele->T_kf_w(2,3)<<" ";
		out<<ele->T_kf_w(3,0)<<" "<<ele->T_kf_w(3,1)<<" "<<ele->T_kf_w(3,2)<<" "<<ele->T_kf_w(3,3)<<endl;
	}
	out.close();
}
void saveStvoTfw(const vector<pair<int,Matrix4d>>& vStvoTfw, string path)
{
	ofstream out(path);
	for(auto ele:vStvoTfw)	//Matrix4d
	{
		out<<"frame_idx "<<ele.first<<" ";
		out<<ele.second(0,0)<<" "<<ele.second(0,1)<<" "<<ele.second(0,2)<<" "<<ele.second(0,3)<<" ";
		out<<ele.second(1,0)<<" "<<ele.second(1,1)<<" "<<ele.second(1,2)<<" "<<ele.second(1,3)<<" ";
		out<<ele.second(2,0)<<" "<<ele.second(2,1)<<" "<<ele.second(2,2)<<" "<<ele.second(2,3)<<" ";
		out<<ele.second(3,0)<<" "<<ele.second(3,1)<<" "<<ele.second(3,2)<<" "<<ele.second(3,3)<<endl;
	}
	out.close();
}