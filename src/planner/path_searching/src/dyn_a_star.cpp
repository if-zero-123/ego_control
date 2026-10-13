#include "path_searching/dyn_a_star.h"
#include <stdexcept>

using namespace std;
using namespace Eigen;

AStar::~AStar()
{
    if (!GridNodeMap_) return;
    for (int i = 0; i < POOL_SIZE_(0); i++)
        for (int j = 0; j < POOL_SIZE_(1); j++)
            for (int k = 0; k < POOL_SIZE_(2); k++)
                delete GridNodeMap_[i][j][k];
    for (int i=0;i<POOL_SIZE_(0);++i) {
        for (int j=0;j<POOL_SIZE_(1);++j) delete[] GridNodeMap_[i][j];
        delete[] GridNodeMap_[i];
    }
    delete[] GridNodeMap_;
}

void AStar::initGridMap(GridMap::Ptr occ_map, const Eigen::Vector3i pool_size)
{
    ros::NodeHandle nh("~");
    nh.param("path_searching/prefer_horizontal",prefer_horizontal_,true);
    nh.param("path_searching/height_band",height_band_,.15);
    nh.param("path_searching/vertical_weight",vertical_weight_,3.);
    nh.param("path_searching/height_weight",height_weight_,.5);
    nh.param("path_searching/total_budget",total_budget_,.2);
    nh.param("path_searching/near_height_budget",near_height_budget_,.06);
    if (!std::isfinite(height_band_) || height_band_<.05 || height_band_>.5 ||
        !std::isfinite(vertical_weight_) || vertical_weight_<1. || vertical_weight_>10. ||
        !std::isfinite(height_weight_) || height_weight_<0. || height_weight_>5. ||
        !std::isfinite(total_budget_) || total_budget_<=0. || total_budget_>.5 ||
        !std::isfinite(near_height_budget_) || near_height_budget_<=0. || near_height_budget_>=total_budget_ ||
        !occ_map || (pool_size.array()<3).any() || GridNodeMap_)
        throw std::invalid_argument("invalid horizontal-first A* configuration");
    POOL_SIZE_ = pool_size;
    CENTER_IDX_ = pool_size / 2;

    GridNodeMap_ = new GridNodePtr **[POOL_SIZE_(0)];
    for (int i = 0; i < POOL_SIZE_(0); i++)
    {
        GridNodeMap_[i] = new GridNodePtr *[POOL_SIZE_(1)];
        for (int j = 0; j < POOL_SIZE_(1); j++)
        {
            GridNodeMap_[i][j] = new GridNodePtr[POOL_SIZE_(2)];
            for (int k = 0; k < POOL_SIZE_(2); k++)
            {
                GridNodeMap_[i][j][k] = new GridNode;
            }
        }
    }

    grid_map_ = occ_map;
}

double AStar::getDiagHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    double h = 0.0;
    int diag = min(min(dx, dy), dz);
    dx -= diag;
    dy -= diag;
    dz -= diag;

    if (dx == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
    }
    if (dy == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
    }
    if (dz == 0)
    {
        h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
    }
    return h;
}

double AStar::getManhHeu(GridNodePtr node1, GridNodePtr node2)
{
    double dx = abs(node1->index(0) - node2->index(0));
    double dy = abs(node1->index(1) - node2->index(1));
    double dz = abs(node1->index(2) - node2->index(2));

    return dx + dy + dz;
}

double AStar::getEuclHeu(GridNodePtr node1, GridNodePtr node2)
{
    return (node2->index - node1->index).norm();
}

vector<GridNodePtr> AStar::retrievePath(GridNodePtr current)
{
    vector<GridNodePtr> path;
    path.push_back(current);

    while (current->cameFrom != NULL)
    {
        current = current->cameFrom;
        path.push_back(current);
    }

    return path;
}

bool AStar::ConvertToIndexAndAdjustStartEndPoints(Vector3d start_pt, Vector3d end_pt, Vector3i &start_idx, Vector3i &end_idx)
{
    if (!Coord2Index(start_pt, start_idx) || !Coord2Index(end_pt, end_idx))
        return false;
    if ((start_pt-end_pt).norm()<1e-9)
        return !checkOccupancy(Index2Coord(start_idx));

    if (checkOccupancy(Index2Coord(start_idx)))
    {
        //ROS_WARN("Start point is insdide an obstacle.");
        do
        {
            start_pt = (start_pt - end_pt).normalized() * step_size_ + start_pt;
            if (!Coord2Index(start_pt, start_idx))
                return false;
        } while (checkOccupancy(Index2Coord(start_idx)));
    }

    if (checkOccupancy(Index2Coord(end_idx)))
    {
        //ROS_WARN("End point is insdide an obstacle.");
        do
        {
            end_pt = (end_pt - start_pt).normalized() * step_size_ + end_pt;
            if (!Coord2Index(end_pt, end_idx))
                return false;
        } while (checkOccupancy(Index2Coord(end_idx)));
    }

    return true;
}

bool AStar::AstarSearch(const double step_size, Vector3d start_pt, Vector3d end_pt)
{
    gridPath_.clear();last_search_stage_=-1;
    if (!GridNodeMap_ || !std::isfinite(step_size) || step_size<=0. ||
        !start_pt.allFinite() || !end_pt.allFinite()) return false;
    const auto began=std::chrono::steady_clock::now();
    const auto deadline=began+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(total_budget_));
    step_size_=step_size;inv_step_size_=1./step_size;center_=(start_pt+end_pt)*.5;
    Vector3i start,end;
    if (!ConvertToIndexAndAdjustStartEndPoints(start_pt,end_pt,start,end)) return false;
    const double reference=std::isfinite(reference_height_)?reference_height_:(start_pt.z()+end_pt.z())*.5;
    if (prefer_horizontal_) {
        // Include actual segment endpoints, so a replan never teleports an aircraft
        // that starts outside the preferred height. This is a SEARCH corridor.
        const double low=std::min(reference,std::min(Index2Coord(start).z(),Index2Coord(end).z()))-height_band_;
        const double high=std::max(reference,std::max(Index2Coord(start).z(),Index2Coord(end).z()))+height_band_;
        const auto near_deadline=std::min(deadline,began+std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(near_height_budget_)));
        if (searchPass(start,end,low,high,reference,near_deadline)) {last_search_stage_=1;return true;}
        ROS_DEBUG("Height-local A* exhausted; trying 3D within existing map constraints");
    }
    if (searchPass(start,end,-inf,inf,reference,deadline)) {
        last_search_stage_=prefer_horizontal_?2:0;return true;
    }
    gridPath_.clear();return false;
}

bool AStar::searchPass(const Vector3i& start,const Vector3i& end,double low,double high,
                       double reference,const std::chrono::steady_clock::time_point& deadline)
{
    if (std::chrono::steady_clock::now()>=deadline) return false;
    ++rounds_;
    openSet_=decltype(openSet_)();
    GridNodePtr first=GridNodeMap_[start.x()][start.y()][start.z()];
    GridNodePtr target=GridNodeMap_[end.x()][end.y()][end.z()];
    target->index=end;first->index=start;first->rounds=rounds_;first->gScore=0.;
    first->fScore=getHeu(first,target);first->state=GridNode::OPENSET;first->cameFrom=nullptr;
    openSet_.push({first,first->gScore,first->fScore});
    while (!openSet_.empty()) {
        if (std::chrono::steady_clock::now()>=deadline) return false;
        const Entry entry=openSet_.top();openSet_.pop();GridNodePtr current=entry.node;
        // Heap entries are immutable. Updating a node's score in-place used to
        // violate heap ordering and could bias the selected detour.
        if (current->rounds!=rounds_ || current->state!=GridNode::OPENSET || entry.g!=current->gScore) continue;
        if (current->index==end) {gridPath_=retrievePath(current);return true;}
        current->state=GridNode::CLOSEDSET;
        const Vector3d from=Index2Coord(current->index);
        for (int dx=-1;dx<=1;++dx) for (int dy=-1;dy<=1;++dy) for (int dz=-1;dz<=1;++dz) {
            if (!dx && !dy && !dz) continue;
            const Vector3i index=current->index+Vector3i(dx,dy,dz);
            if ((index.array()<1).any() || (index.array()>=POOL_SIZE_.array()-1).any()) continue;
            const Vector3d to=Index2Coord(index);
            if (to.z()<low || to.z()>high || checkOccupancy(to)) continue;
            GridNodePtr next=GridNodeMap_[index.x()][index.y()][index.z()];
            const bool explored=next->rounds==rounds_;
            if (explored && next->state==GridNode::CLOSEDSET) continue;
            // Check edges as well as lattice vertices: 10cm A* steps can skip
            // occupied 5cm cells or cut diagonal obstacle corners.
            bool blocked=false;
            const int pieces=std::max(2,int(std::ceil((to-from).norm()/(grid_map_->getResolution()*.5))));
            for (int j=1;j<pieces;++j) if (checkOccupancy(from+(to-from)*(double(j)/pieces))) {blocked=true;break;}
            if (blocked) continue;
            const double vertical=prefer_horizontal_?vertical_weight_:1.;
            const double distance=std::sqrt(dx*dx+dy*dy+vertical*vertical*dz*dz);
            const double deviation=std::max(0.,std::abs(to.z()-reference)-step_size_*.5);
            const double cost=distance*(1.+(prefer_horizontal_?height_weight_*deviation/step_size_:0.));
            const double score=current->gScore+cost;
            if (!explored || score<next->gScore) {
                next->index=index;next->rounds=rounds_;next->state=GridNode::OPENSET;
                next->cameFrom=current;next->gScore=score;next->fScore=score+getHeu(next,target);
                openSet_.push({next,next->gScore,next->fScore});
            }
        }
    }
    return false;
}
vector<Vector3d> AStar::getPath()
{
    vector<Vector3d> path;

    for (auto ptr : gridPath_)
        path.push_back(Index2Coord(ptr->index));

    reverse(path.begin(), path.end());
    return path;
}
