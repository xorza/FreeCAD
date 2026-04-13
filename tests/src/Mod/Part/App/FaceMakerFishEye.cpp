// SPDX-License-Identifier: LGPL-2.1-or-later

#include <gtest/gtest.h>
#include "src/App/InitApplication.h"
#include <App/Application.h>
#include <App/Document.h>
#include "Mod/Part/App/FaceMakerFishEye.h"
#include "Mod/Part/App/TopoShape.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeCircle.hxx>
#include <Geom_BSplineCurve.hxx>
#include <GeomAPI_Interpolate.hxx>
#include <gp_Pln.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>

#include <numbers>

// NOLINTBEGIN(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)

using namespace Part;

class FaceMakerFishEyeTest: public ::testing::Test
{
protected:
    static void SetUpTestSuite()
    {
        tests::initApplication();
    }

    void SetUp() override
    {
        _docName = App::GetApplication().getUniqueDocumentName("test");
        App::GetApplication().newDocument(_docName.c_str(), "testUser");
        _doc = App::GetApplication().getDocument(_docName.c_str());
    }

    void TearDown() override
    {
        App::GetApplication().closeDocument(_docName.c_str());
    }

    static TopoDS_Wire makeRectWire(double x0, double y0, double x1, double y1)
    {
        gp_Pnt p0(x0, y0, 0), p1(x1, y0, 0), p2(x1, y1, 0), p3(x0, y1, 0);
        BRepBuilderAPI_MakeWire mw;
        mw.Add(BRepBuilderAPI_MakeEdge(p0, p1).Edge());
        mw.Add(BRepBuilderAPI_MakeEdge(p1, p2).Edge());
        mw.Add(BRepBuilderAPI_MakeEdge(p2, p3).Edge());
        mw.Add(BRepBuilderAPI_MakeEdge(p3, p0).Edge());
        return mw.Wire();
    }

    static TopoDS_Wire makeLineWire(double x0, double y0, double x1, double y1)
    {
        return BRepBuilderAPI_MakeWire(
            BRepBuilderAPI_MakeEdge(gp_Pnt(x0, y0, 0), gp_Pnt(x1, y1, 0)).Edge()
        ).Wire();
    }

    static TopoDS_Wire makeFigure8Wire()
    {
        TColgp_Array1OfPnt points(1, 9);
        points.SetValue(1, gp_Pnt(0, 0, 0));
        points.SetValue(2, gp_Pnt(5, 5, 0));
        points.SetValue(3, gp_Pnt(10, 0, 0));
        points.SetValue(4, gp_Pnt(5, -5, 0));
        points.SetValue(5, gp_Pnt(0, 0, 0));
        points.SetValue(6, gp_Pnt(-5, -5, 0));
        points.SetValue(7, gp_Pnt(-10, 0, 0));
        points.SetValue(8, gp_Pnt(-5, 5, 0));
        points.SetValue(9, gp_Pnt(0, 0, 0));
        Handle(TColgp_HArray1OfPnt) hpoints = new TColgp_HArray1OfPnt(points);
        GeomAPI_Interpolate interp(hpoints, Standard_False, Precision::Confusion());
        interp.Perform();
        auto edge = BRepBuilderAPI_MakeEdge(interp.Curve()).Edge();
        return BRepBuilderAPI_MakeWire(edge).Wire();
    }

    /// Call makeElementFace with FishEye and source TopoShapes that have
    /// proper Tags so that element naming is exercised.
    TopoShape makeFishEyeFace(const std::vector<TopoDS_Wire>& wires)
    {
        auto hasher = _doc->getStringHasher();
        long tag = 1;

        // Wrap each wire in a TopoShape with a unique tag and element map
        std::vector<TopoShape> sources;
        for (const auto& w : wires) {
            TopoShape ts(tag++, hasher);
            ts.setShape(w);
            ts.mapSubElement(ts);
            sources.push_back(std::move(ts));
        }

        TopoShape result(tag, hasher);
        result.makeElementFace(sources, nullptr, "Part::FaceMakerFishEye", nullptr);
        return result;
    }

    std::string _docName;
    App::Document* _doc = nullptr;
};

// Rectangle + diagonal: edges split at intersection, no fuseOverlaps involved.
// This tests the mySplitter history tracking in buildPlanar.
TEST_F(FaceMakerFishEyeTest, subdivisionEdgesHaveMappedNames)
{
    auto result = makeFishEyeFace({makeRectWire(0, 0, 10, 10), makeLineWire(0, 0, 10, 10)});

    // Diagonal splits rectangle into 2 triangles
    ASSERT_GE(result.countSubShapes(TopAbs_FACE), 2);

    int edgeCount = result.countSubShapes(TopAbs_EDGE);
    ASSERT_GT(edgeCount, 0);
    for (int i = 1; i <= edgeCount; ++i) {
        auto indexed = Data::IndexedName::fromConst("Edge", i);
        auto mapped = result.getMappedName(indexed);
        EXPECT_TRUE(mapped) << "Edge" << i << " has no mapped name";
        if (mapped) {
            std::string mappedStr = mapped.toString();
            std::string indexedStr = indexed.toString();
            EXPECT_NE(mappedStr, indexedStr)
                << "Edge" << i << " has identity mapping (unnamed)";
        }
    }
}

TEST_F(FaceMakerFishEyeTest, subdivisionNamingStable)
{
    auto result1 = makeFishEyeFace({makeRectWire(0, 0, 10, 10), makeLineWire(0, 0, 10, 10)});
    auto result2 = makeFishEyeFace({makeRectWire(0, 0, 10, 10), makeLineWire(0, 0, 10, 10)});

    int edgeCount = result1.countSubShapes(TopAbs_EDGE);
    ASSERT_EQ(edgeCount, result2.countSubShapes(TopAbs_EDGE));
    for (int i = 1; i <= edgeCount; ++i) {
        auto indexed = Data::IndexedName::fromConst("Edge", i);
        auto mapped1 = result1.getMappedName(indexed);
        auto mapped2 = result2.getMappedName(indexed);
        EXPECT_EQ(mapped1, mapped2) << "Edge" << i << " name differs across builds";
    }
}

// Two overlapping rectangles: fuseOverlaps creates new edges via BRepAlgoAPI_Fuse.
// The fuse history must trace fused edges back to the original rectangle edges.
TEST_F(FaceMakerFishEyeTest, fusedOverlapEdgesHaveMappedNames)
{
    auto result = makeFishEyeFace({makeRectWire(0, 0, 20, 20), makeRectWire(10, 10, 30, 30)});

    ASSERT_GE(result.countSubShapes(TopAbs_FACE), 1);

    int edgeCount = result.countSubShapes(TopAbs_EDGE);
    ASSERT_GT(edgeCount, 0);
    for (int i = 1; i <= edgeCount; ++i) {
        auto indexed = Data::IndexedName::fromConst("Edge", i);
        auto mapped = result.getMappedName(indexed);
        EXPECT_TRUE(mapped) << "Edge" << i << " has no mapped name";
        if (mapped) {
            std::string mappedStr = mapped.toString();
            std::string indexedStr = indexed.toString();
            EXPECT_NE(mappedStr, indexedStr)
                << "Edge" << i << " has identity mapping (unnamed)";
        }
    }
}

TEST_F(FaceMakerFishEyeTest, splitBSplineEdgesHaveMappedNames)
{
    auto result = makeFishEyeFace({makeFigure8Wire()});

    // Figure-8 should produce at least 2 faces
    ASSERT_GE(result.countSubShapes(TopAbs_FACE), 2);

    int edgeCount = result.countSubShapes(TopAbs_EDGE);
    ASSERT_GT(edgeCount, 0);
    for (int i = 1; i <= edgeCount; ++i) {
        auto indexed = Data::IndexedName::fromConst("Edge", i);
        auto mapped = result.getMappedName(indexed);
        EXPECT_TRUE(mapped) << "Edge" << i << " has no mapped name";
        if (mapped) {
            std::string mappedStr = mapped.toString();
            std::string indexedStr = indexed.toString();
            EXPECT_NE(mappedStr, indexedStr)
                << "Edge" << i << " has identity mapping (unnamed)";
        }
    }
}

TEST_F(FaceMakerFishEyeTest, splitBSplineNamingStable)
{
    auto result1 = makeFishEyeFace({makeFigure8Wire()});
    auto result2 = makeFishEyeFace({makeFigure8Wire()});

    int edgeCount = result1.countSubShapes(TopAbs_EDGE);
    ASSERT_EQ(edgeCount, result2.countSubShapes(TopAbs_EDGE));
    for (int i = 1; i <= edgeCount; ++i) {
        auto indexed = Data::IndexedName::fromConst("Edge", i);
        auto mapped1 = result1.getMappedName(indexed);
        auto mapped2 = result2.getMappedName(indexed);
        EXPECT_EQ(mapped1, mapped2) << "Edge" << i << " name differs across builds";
    }
}

// NOLINTEND(readability-magic-numbers,cppcoreguidelines-avoid-magic-numbers)
