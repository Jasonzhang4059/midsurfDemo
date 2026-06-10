//#pragma once
//
//#include <vector>
//#include <list>
//#include <queue>
//#include <string>
//#include <stack>
//
//#include <TopoDS_Face.hxx>
//#include <TopoDS_Wire.hxx>
//#include <TopoDS_Edge.hxx>
//#include <TopoDS_Vertex.hxx>
//#include <TopoDS.hxx>
//
//#include <TopExp.hxx>
//#include <BRep_Tool.hxx>
//
//#include <gp_Pnt.hxx>
//
//// ======================================================
//// OCC 版 SfCtainTreeNode
//// 对应：
////   Spline        -> TopoDS_Edge
////   SplineSurface -> TopoDS_Face
////   Vec3          -> gp_Pnt
//// ======================================================
//class SfCtainTreeNode
//{
//public:
//    SfCtainTreeNode();
//    SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines);
//    SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines,
//        const std::vector<TopoDS_Edge>& conlines);
//    SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines,
//        const std::vector<TopoDS_Edge>& conlines,
//        const std::vector<int>& seg);
//    SfCtainTreeNode(const std::vector<TopoDS_Edge>& outlines,
//        const std::vector<TopoDS_Edge>& conlines,
//        const std::vector<int>& seg,
//        bool genus);
//    ~SfCtainTreeNode();
//
//public:
//    void GetOutPoints(std::vector<gp_Pnt>& ps) const;
//    void GetOutLines(std::vector<TopoDS_Edge>& outline) const;
//    void GetSurfs(std::vector<TopoDS_Face>& outSurfs) const;
//
//    void AddChild(SfCtainTreeNode* child);
//    void AddSurf(const TopoDS_Face& face);
//    void UpdateAllLines();
//    void UpdateVertexFromOutLines();
//    void Clear();
//
//private:
//    bool GetEdgeStartPoint(const TopoDS_Edge& edge, gp_Pnt& p) const;
//
//public:
//    std::vector<TopoDS_Edge> outLines;                // 外围轮廓线
//    std::vector<TopoDS_Edge> conLines;                // 内外连接线
//    std::vector<gp_Pnt> vertex;                       // 外轮廓顶点坐标
//    std::vector<int> isSeg;                           // 外轮廓曲线的可分割性
//    bool isGenus;                                     // 是否亏格
//
//    std::vector<TopoDS_Edge> allLines;                // 所有曲线
//    std::vector<int> outNumber;                       // 外围轮廓线序号
//    std::vector<std::vector<int>> quadPolNumber;      // 所有四边形对应曲线序号
//    std::list<SfCtainTreeNode*> childs;               // 子节点
//    bool isSort;                                      // 顶点是否经过排序
//    int num;                                          // 曲面序号
//    bool quaded;                                      // 是否完成剖分
//
//    std::vector<TopoDS_Face> surfs;                   // 当前节点剖分得到的曲面
//};
//
//// ======================================================
//// occMidSurfQuad
//// ======================================================
//class OccMidSurfQuad
//{
//public:
//    OccMidSurfQuad();
//    ~OccMidSurfQuad();
//
//public:
//    // 参考你给的 quadSurface()
//    std::vector<TopoDS_Face> QuadSurface();
//
//    // 单独对输入面做四边剖分
//    bool QuadFaces(const std::vector<TopoDS_Face>& m_fac,
//        std::vector<TopoDS_Face>& allSurf);
//
//    // 对应原来的：
//    // void quad(varray<Spline>& outer, varray<varray<Spline>>& inner,
//    //           varray<bool>& genus, varray<SplineSurface>& allSurf)
//    void quad(std::vector<TopoDS_Edge>& outer,
//        std::vector<std::vector<TopoDS_Edge>>& inner,
//        std::vector<bool>& genus,
//        std::vector<TopoDS_Face>& allSurf);
//
//private:
//    // ===== 这些函数在你上传的文件里出现过名字/调用关系 =====
//    // 若后续你有原始实现，再继续替换成 OCC 版完整逻辑
//
//    void unfoldMeshSurface(const std::string& path, std::vector<gp_Pnt>& vecs);
//
//    bool getMeshModelBoundary(const std::string& meshFile,
//        std::vector<std::vector<TopoDS_Edge>>& boundRes);
//
//    bool quadPlane(const std::vector<std::vector<TopoDS_Edge>>& boundRes,
//        const std::vector<bool>& genus,
//        std::vector<std::vector<gp_Pnt>>& bV);
//
//    std::vector<TopoDS_Face> fittingSurface(std::vector<std::vector<TopoDS_Edge>>& boundRes,
//        int uDegree,
//        int uNum);
//
//    bool createAddline(std::vector<std::vector<TopoDS_Edge>>& surf,
//        std::vector<TopoDS_Edge>& addLines);
//
//    SfCtainTreeNode* CreateSurfContainTree(const std::vector<std::vector<TopoDS_Edge>>& surf,
//        const std::vector<TopoDS_Edge>& addLines,
//        const std::vector<std::vector<int>>& seg,
//        std::vector<bool>& genus);
//
//    void QuadWithContainTree(SfCtainTreeNode* root);
//
//    void getAllSurface(SfCtainTreeNode* root,
//        std::vector<TopoDS_Face>& allSurf) const;
//
//    void quadAdjustUV(std::vector<TopoDS_Face>& allSurf) const;
//
//private:
//    // 面提取轮廓辅助
//    bool ExtractFaceLoops(const TopoDS_Face& face,
//        std::vector<TopoDS_Edge>& outer,
//        std::vector<std::vector<TopoDS_Edge>>& inner);
//};