#include "tesselation.h"

// make normals for each face - duplicates all vertex data
void facet_normals(Mesh* mesh) {
    // allocates new arrays
    auto pos = vector<vec3f>();
    auto norm = vector<vec3f>();
    auto texcoord = vector<vec2f>();
    auto triangle = vector<vec3i>();
    auto quad = vector<vec4i>();
    // froeach triangle
    for(auto f : mesh->triangle) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face face normal
        auto fn = normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x]));
        // add triangle
        triangle.push_back({nv,nv+1,nv+2});
        // add vertex data
        for(auto i : range(3)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(not mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }
    // froeach quad
    for(auto f : mesh->quad) {
        // grab current pos size
        auto nv = (int)pos.size();
        // compute face normal
        auto fn = normalize(normalize(cross(mesh->pos[f.y]-mesh->pos[f.x], mesh->pos[f.z]-mesh->pos[f.x])) +
                            normalize(cross(mesh->pos[f.z]-mesh->pos[f.x], mesh->pos[f.w]-mesh->pos[f.x])));
        // add quad
        quad.push_back({nv,nv+1,nv+2,nv+3});
        // add vertex data
        for(auto i : range(4)) {
            pos.push_back(mesh->pos[f[i]]);
            norm.push_back(fn);
            if(not mesh->texcoord.empty()) texcoord.push_back(mesh->texcoord[f[i]]);
        }
    }
    // set back mesh data
    mesh->pos = pos;
    mesh->norm = norm;
    mesh->texcoord = texcoord;
    mesh->triangle = triangle;
    mesh->quad = quad;
}

// smooth out normal - does not duplicate data
void smooth_normals(Mesh* mesh) {
    // YOUR CODE GOES HERE ---------------------
    // set normals array to the same length as pos and init all elements to zero
    auto normals = vector<vec3f>(mesh->pos.size(), zero3f);
    // foreach triangle
    for (auto triangle: mesh->triangle) {
        // compute face normal
        vec3f normal = zero3f;
        vec3f U, V;
        U = mesh->pos[triangle.y]-mesh->pos[triangle.x];
        V = mesh->pos[triangle.z]-mesh->pos[triangle.x];
        normal = normalize(cross(U, V));
        
        // accumulate face normal to the vertex normals of each face index
        normals[triangle.x] += normal;
        normals[triangle.y] += normal;
        normals[triangle.z] += normal;
    }
    // foreach quad
    for (auto quad: mesh->quad) {
        // compute face normal
        vec3f normal1 = zero3f;
        vec3f normal2 = zero3f;
        vec3f normal = zero3f;
        vec3f U, V, T, S;
        U = mesh->pos[quad.y]-mesh->pos[quad.x];
        V = mesh->pos[quad.z]-mesh->pos[quad.x];
        T = mesh->pos[quad.y]-mesh->pos[quad.w];
        S = mesh->pos[quad.z]-mesh->pos[quad.w];
        normal1 = normalize(cross(U, V));
        normal2 = normalize(cross(T, S));
        normal = normalize((normal1+normal2)/2);
        
        // accumulate face normal to the vertex normals of each face index
        normals[quad.x] += normal;
        normals[quad.y] += normal;
        normals[quad.z] += normal;
        normals[quad.w] += normal;
    }
    // normalize all vertex normals
    for(vec3f norm : normals){
        norm = normalize(norm);
    }
}

// smooth out tangents
void smooth_tangents(Mesh* polyline) {
    // set tangent array
    polyline->norm = vector<vec3f>(polyline->pos.size(),zero3f);
    // foreach line
    for(auto l : polyline->line) {
        // compute line tangent
        auto lt = normalize(polyline->pos[l.y]-polyline->pos[l.x]);
        // accumulate segment tangent to vertex tangent on each vertex
        for (auto i : range(2)) polyline->norm[l[i]] += lt;
    }
    // normalize all vertex tangents
    for (auto& t : polyline->norm) t = normalize(t);
}

// apply Catmull-Clark mesh subdivision
// does not subdivide texcoord
void subdivide_catmullclark(Mesh* subdiv) {
    // YOUR CODE GOES HERE ---------------------
    // skip is needed
    if (subdiv->subdivision_catmullclark_level == 0) {
        return;
    }
    // allocate a working Mesh copied from the subdiv
    Mesh* copy = new Mesh(*subdiv);
    // foreach level
    for(int i = 0; i< copy->subdivision_catmullclark_level; i++){
        // make empty pos and quad arrays
        auto new_pos = vector<vec3f>();
        auto new_quad = vector<vec4i>(copy->triangle.size()*3+copy->quad.size()*4, zero4i);
        // create edge_map from current mesh
        auto edge_map = EdgeMap(copy->triangle, copy->quad);
        
        // linear subdivision - create vertices
        // copy all vertices from the current mesh
        new_pos = copy->pos;
        // add vertices in the middle of each edge (use EdgeMap)
        auto edge_list = edge_map.edges();
        auto mid_points = vector<vec3f>(edge_list.size(),zero3f);
        
        for (auto edge: edge_list){
            int p1 = edge.x;
            int p2 = edge.y;
            mid_points[edge_map.edge_index(edge)] = (copy->pos[p1] + copy->pos[p2])/2;
        }
        // add vertices in the middle of each triangle
        auto triangle_centroid = vector<vec3f>(copy->triangle.size(),zero3f);
        i = 0;
        for (auto triangle: copy->triangle){
            triangle_centroid[i] = (copy->pos[triangle.x]+copy->pos[triangle.y]+copy->pos[triangle.z])/3;
            i++;
        }
        // add vertices in the middle of each quad
        auto quad_centroid = vector<vec3f>(copy->quad.size(),zero3f);
        i = 0;
        for (auto quad: copy->quad){
            quad_centroid[i] = (copy->pos[quad.x]+copy->pos[quad.y]+copy->pos[quad.z]+copy->pos[quad.w])/4;
            i++;
        }
        
        new_pos.insert(new_pos.end(), mid_points.begin(), mid_points.end());
        new_pos.insert(new_pos.end(), triangle_centroid.begin(), triangle_centroid.end());
        new_pos.insert(new_pos.end(), quad_centroid.begin(), quad_centroid.end());
        
        // subdivision pass --------------------------------
        // compute an offset for the edge vertices
        int mid_points_offset = copy->pos.size();
        // compute an offset for the triangle vertices
        int triangle_offset = mid_points_offset + mid_points.size();
        // compute an offset for the quad vertices
        int quad_offset = triangle_offset + triangle_centroid.size();
        // foreach triangle
        i = 0;
        for(auto triangle : copy->triangle){
            // add three quads to the new quad array
            int mid_point1, mid_point2, mid_point3;
            
            mid_point1 = edge_map.edge_index(vec2i(i, i+1));
            mid_point2 = edge_map.edge_index(vec2i(i, i+2));
            mid_point3 = edge_map.edge_index(vec2i(i+1, i+2));
            
            new_quad[i] = vec4i(i, mid_points_offset+mid_point1, triangle_offset+i, mid_points_offset+mid_point2);
            new_quad[i+1] = vec4i(i+1, mid_points_offset+mid_point3, triangle_offset+i, mid_points_offset+mid_point1);
            new_quad[i+2] = vec4i(i+2, mid_points_offset+mid_point2, triangle_offset+i, mid_points_offset+mid_point3);
            
            i += 3;
        }
        // foreach quad
        
        for (auto quad: copy->quad){
            int mid_point1, mid_point2, mid_point3, mid_point4;
            
            mid_point1 = edge_map.edge_index(vec2i(i, i+1));
            mid_point2 = edge_map.edge_index(vec2i(i+1, i+2));
            mid_point3 = edge_map.edge_index(vec2i(i+2, i+3));
            mid_point4 = edge_map.edge_index(vec2i(i+3, i));
            // add four quads to the new quad array
            new_quad[i] = vec4i(i, mid_points_offset+mid_point1, quad_offset+i, mid_points_offset+mid_point4);
            new_quad[i+1] = vec4i(i+1, mid_points_offset+mid_point2, quad_offset+i, mid_points_offset+mid_point1);
            new_quad[i+2] = vec4i(i+2, mid_points_offset+mid_point3, quad_offset+i, mid_points_offset+mid_point2);
            new_quad[i+3] = vec4i(i+3, mid_points_offset+mid_point4, quad_offset+i, mid_points_offset+mid_point3);
            i += 4;
        }
        // averaging pass ----------------------------------
        // create arrays to compute pos averages (avg_pos, avg_count)
        // arrays have the same length as the new pos array, and are init to zero
        auto avg_pos = vector<vec3f> (new_pos.size(), zero3f);
        auto avg_count = vector<unsigned int> (new_pos.size(), 0);
        // for each new quad
        for(auto quad: new_quad){
            // compute quad center using the new pos array
            auto c = (new_pos[quad.x]+new_pos[quad.y]+new_pos[quad.z]+new_pos[quad.w])/4;
            // foreach vertex index in the quad
            for (int i = 0; i < 4; i++) {
                int v_index = quad[i];
                avg_pos[v_index] += c;
                avg_count[v_index] += 1 ;
            }
        }
        
        // normalize avg_pos with its count avg_count
        for (int i = 0; i < avg_pos.size(); i++)
            avg_pos[i] = avg_pos[i]/avg_count[i];
        
        // correction pass ----------------------------------
        // foreach pos, compute correction p = p + (avg_p - p) * (4/avg_count)
        for (int v_index = 0; v_index < avg_pos.size(); v_index++)
            new_pos[v_index] += (avg_pos[v_index] - new_pos[v_index] * 4 / avg_count[v_index]);
            
        // set new arrays pos, quad back into the working mesh; clear triangle array
        copy->pos.clear();
//        copy->pos.insert(copy->pos.end(), new_pos.begin(),new_pos.end());
        copy->pos = vector<vec3f>(new_pos.size(), zero3f);
        copy->pos = new_pos;
        
        copy->quad.clear();
        copy->quad = vector<vec4i>(new_quad.size(), zero4i);
        copy->quad = new_quad;
        copy->triangle.clear();
    }
    // clear subdivision
    subdiv->pos.clear();
    subdiv->quad.clear();
    subdiv->triangle.clear();
    // according to smooth, either smooth_normals or facet_normals
    facet_normals(copy);
    // copy back
    subdiv = copy;
    // clear
    delete [] &copy;
    
}

// subdivide bezier spline into line segments (assume bezier has only bezier segments and no lines)
void subdivide_bezier(Mesh* bezier) {
    // YOUR CODE GOES HERE ---------------------
    // skip is needed
    // allocate a working polyline from bezier
    Mesh* polyline = bezier;
    // foreach level
    for (int i = 0; i < bezier->subdivision_bezier_level; i++){
        // make new arrays of positions and bezier segments
        auto pos = vector<vec3f>();
        auto segments = vector<int>();
        // copy all the vertices into the new array (this waste space but it is easier for now)
        // foreach bezier segment
            // apply subdivision algorithm
            // prepare indices for two new segments
            // add mid point
            // add points for first segment and fix segment indices
            // add points for second segment and fix segment indices
            // add indices for both segments into new segments array
        // set new arrays pos, segments into the working lineset
    }
    // copy bezier segments into line segments
    // clear bezier array from lines
    // run smoothing to get proper tangents
    // copy back
    // clear
}

Mesh* make_surface_mesh(frame3f frame, float radius, bool isquad, Material* mat, float offset) {
    auto mesh = new Mesh{};
    mesh->frame = frame;
    mesh->mat = mat;
    if(isquad) {
        mesh->pos = { {-radius,-radius,-offset}, {radius,-radius,-offset},
            {radius,radius,-offset}, {-radius,radius,-offset} };
        mesh->norm = {z3f,z3f,z3f,z3f};
        mesh->quad = { {0,1,2,3} };
    } else {
        map<pair<int,int>,int> vid;
        for(auto j : range(64+1)) {
            for(auto i : range(128+1)) {
                auto u = 2 * pif * i / 64.0f, v = pif * j / 32.0f;
                auto d = vec3f{cos(u)*sin(v),sin(u)*sin(v),cos(v)};
                vid[{i,j}] = mesh->pos.size();
                mesh->pos.push_back(d*radius*(1-offset));
                mesh->norm.push_back(d);
            }
        }
        for(auto j : range(64)) {
            for(auto i : range(128)) {
                mesh->quad.push_back({vid[{i,j}],vid[{i+1,j}],vid[{i+1,j+1}],vid[{i,j+1}]});
            }
        }
    }
    return mesh;
}

void subdivide_surface(Surface* surface) {
    surface->_display_mesh = make_surface_mesh(
        surface->frame, surface->radius, surface->isquad, surface->mat);
}

void subdivide(Scene* scene) {
    for(auto mesh : scene->meshes) {
        if(mesh->subdivision_catmullclark_level) subdivide_catmullclark(mesh);
        if(mesh->subdivision_bezier_level) subdivide_bezier(mesh);
    }
    for(auto surface : scene->surfaces) {
        subdivide_surface(surface);
    }
}
