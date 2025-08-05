import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:tuple/tuple.dart';

import '../database/dbimage.dart';
import '../hardware/models/comm_code.dart';
import '../model.dart';
import '../widgets/connection_state_indicator.dart';
import '../widgets/pattern_import_button.dart';
import './create.dart';

class MyHomePage extends StatefulWidget {
  const MyHomePage({super.key});

  @override
  State<MyHomePage> createState() => _MyHomePageState();
}

class _MyHomePageState extends State<MyHomePage> {
  final ValueNotifier<bool> loading = ValueNotifier<bool>(false);
  int tabIndex = 0;
  int _selectedPoiIndex = 0; // New variable to track the selected device
  final ScrollController _scrollController = ScrollController();

  @override
  Widget build(BuildContext context) {
    final model = Provider.of<Model>(context);
    return Scaffold(
      appBar: AppBar(
        title: const Text("Open Pixel Poi"),
        actions: [
          // Dropdown to select a specific device
          if (model.connectedPoi!.length > 1)
            Padding(
              padding: const EdgeInsets.symmetric(horizontal: 16.0),
              child: DropdownButton<int>(
                value: _selectedPoiIndex,
                items: List.generate(
                  model.connectedPoi!.length,
                  (index) => DropdownMenuItem(
                    value: index,
                    child: Text('Device ${index + 1}'),
                  ),
                ),
                onChanged: (int? newValue) {
                  setState(() {
                    _selectedPoiIndex = newValue!;
                  });
                },
              ),
            ),
          ...model.connectedPoi!
            .map((e) => ConnectionStateIndicator(model.connectedPoi!.indexOf(e)))
        ],
      ),
      body: Stack(
        children: [
          getButtons(context),
          getLoading(context),
        ],
      ),
    );
  }

  Widget getLoading(BuildContext buildContext) {
    return ValueListenableBuilder<bool>(
        valueListenable: loading,
        builder: (BuildContext context, bool value, Widget? child) {
          if (!value) {
            return const SizedBox.shrink();
          }
          return Container(
            color: Colors.black38,
            child: Center(
              child: Container(
                color: Colors.white,
                child: Padding(
                  padding: const EdgeInsets.all(20),
                  child: Column(
                    mainAxisSize: MainAxisSize.min,
                    children: const [
                      Text(
                        "Transmitting Pattern...",
                        textAlign: TextAlign.center,
                        style: TextStyle(
                          fontSize: 24,
                          fontWeight: FontWeight.bold,
                        ),
                      ),
                      SizedBox(
                        height: 30,
                      ),
                      CircularProgressIndicator(),
                    ],
                  ),
                ),
              ),
            ),
          );
        });
  }

  Widget getButtons(BuildContext buildContext) {
    return Column(
      children: [
        getPrimarySettings(buildContext),
        Expanded(
          child: getImagesList(buildContext),
        ),
      ],
    );
  }

  Widget getPrimarySettings(BuildContext buildContext) {
    return DefaultTabController(
      initialIndex: 0,
      length: 4,
      child: Column(
        children: [
          TabBar(
            onTap: (index) {
              setState(() {
                tabIndex = index;
              });
            },
            tabs: const [
              Tab(
                icon: Icon(
                  Icons.blur_linear,
                  color: Colors.blue,
                ),
              ),
              Tab(
                icon: Icon(
                  Icons.attractions,
                  color: Colors.blue,
                ),
              ),
              Tab(
                icon: Icon(
                  Icons.brightness_6,
                  color: Colors.blue,
                ),
              ),
              Tab(
                icon: Icon(
                  Icons.sixty_fps_select_rounded,
                  color: Colors.blue,
                ),
              ),
            ],
          ),
          if (tabIndex == 1) getPatternSots(buildContext),
          if (tabIndex == 2) getBrightnessButtons(buildContext),
          if (tabIndex == 3) getFrequencyButtons(buildContext),
        ],
      ),
    );
  }

  Widget getBrightnessButtons(BuildContext buildContext) {
    return Card(
      elevation: 5,
      child: Padding(
        padding: const EdgeInsets.only(top: 8.0),
        child: ListTile(
          title: const Text("Brightness Level",
            style: TextStyle(
              color: Colors.blue,
              fontSize: 24,
              fontWeight: FontWeight.bold,
            )),
          subtitle: Column(
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("1",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(1, CommCode.CC_SET_BRIGHTNESS, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("2",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(4, CommCode.CC_SET_BRIGHTNESS, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("3",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(10, CommCode.CC_SET_BRIGHTNESS, false),
                  ),
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("4",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(25, CommCode.CC_SET_BRIGHTNESS, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("5",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(50, CommCode.CC_SET_BRIGHTNESS, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("6",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(100, CommCode.CC_SET_BRIGHTNESS, false),
                  ),
                ],
              ),
            ],
          ),
        ),
      ),
    );
  }

  Widget getFrequencyButtons(BuildContext buildContext) {
    return Card(
      elevation: 5,
      child: Padding(
        padding: const EdgeInsets.only(top: 8.0),
        child: ListTile(
          title: const Text("Frames Per Second",
            style: TextStyle(
              color: Colors.blue,
              fontSize: 24,
              fontWeight: FontWeight.bold,
            )),
          subtitle: Column(
            children: [
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("0",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(0, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("2",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(1, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("4",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(2, CommCode.CC_SET_SPEED, false),
                  ),
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("10",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(5, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("20",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(10, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("40",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(20, CommCode.CC_SET_SPEED, false),
                  ),
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("100",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(50, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("150",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(75, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("200",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(100, CommCode.CC_SET_SPEED, false),
                  ),
                ],
              ),
              Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("300",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(150, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("400",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(200, CommCode.CC_SET_SPEED, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("500",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(250, CommCode.CC_SET_SPEED, false),
                  ),
                ],
              ),
            ],
          ),
        ),
    );
  }

  Widget getPatternSots(BuildContext buildContext) {
    return Card(
      elevation: 5,
      child: Padding(
        padding: const EdgeInsets.only(top: 8.0),
        child: Column(
          children: [
            ListTile(
              title: const Text("Pattern Bank",
                style: TextStyle(
                  color: Colors.blue,
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                )),
              subtitle: Row(
                mainAxisAlignment: MainAxisAlignment.spaceBetween,
                children: [
                  ElevatedButton(
                    child: const Text("1",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(0, CommCode.CC_SET_BANK, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("2",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(1, CommCode.CC_SET_BANK, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("3",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendInt8(2, CommCode.CC_SET_BANK, false),
                  ),
                  const VerticalDivider(width: 8.0),
                  ElevatedButton(
                    child: const Text("∞",
                        style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                    onPressed: () => Provider.of<Model>(context, listen: false)
                        .connectedPoi![_selectedPoiIndex]
                        .sendCommCode(CommCode.CC_SET_BANK_ALL, false),
                  ),
                ],
              ),
            ),
            ListTile(
              title: const Text("Pattern Slot",
                style: TextStyle(
                  color: Colors.blue,
                  fontSize: 24,
                  fontWeight: FontWeight.bold,
                )),
              subtitle: Column(
                children: [
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      ElevatedButton(
                        child: const Text("1",
                            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                        onPressed: () => Provider.of<Model>(context, listen: false)
                            .connectedPoi![_selectedPoiIndex]
                            .sendInt8(0, CommCode.CC_SET_PATTERN_SLOT, false),
                      ),
                      const VerticalDivider(width: 8.0),
                      ElevatedButton(
                        child: const Text("2",
                            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                        onPressed: () => Provider.of<Model>(context, listen: false)
                            .connectedPoi![_selectedPoiIndex]
                            .sendInt8(1, CommCode.CC_SET_PATTERN_SLOT, false),
                      ),
                      const VerticalDivider(width: 8.0),
                      ElevatedButton(
                        child: const Text("3",
                            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                        onPressed: () => Provider.of<Model>(context, listen: false)
                            .connectedPoi![_selectedPoiIndex]
                            .sendInt8(2, CommCode.CC_SET_PATTERN_SLOT, false),
                      ),
                    ],
                  ),
                  Row(
                    mainAxisAlignment: MainAxisAlignment.spaceBetween,
                    children: [
                      ElevatedButton(
                        child: const Text("4",
                            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                        onPressed: () => Provider.of<Model>(context, listen: false)
                            .connectedPoi![_selectedPoiIndex]
                            .sendInt8(3, CommCode.CC_SET_PATTERN_SLOT, false),
                      ),
                      const VerticalDivider(width: 8.0),
                      ElevatedButton(
                        child: const Text("5",
                            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                        onPressed: () => Provider.of<Model>(context, listen: false)
                            .connectedPoi![_selectedPoiIndex]
                            .sendInt8(4, CommCode.CC_SET_PATTERN_SLOT, false),
                      ),
                      const VerticalDivider(width: 8.0),
                      ElevatedButton(
                        child: const Text("∞",
                            style: TextStyle(fontSize: 24, fontWeight: FontWeight.bold)),
                        onPressed: () => Provider.of<Model>(context, listen: false)
                            .connectedPoi![_selectedPoiIndex]
                            .sendCommCode(CommCode.CC_SET_PATTERN_ALL, false),
                      ),
                    ],
                  ),
                ],
              ),
            ),
          ],
        ),
      ),
    );
  }

  Widget getImagesList(BuildContext buildContext) {
    return Card(
      elevation: 5,
      child: ListTile(
        title: Row(
          mainAxisAlignment: MainAxisAlignment.spaceBetween,
          children: [
            const Text(
              'Patterns',
              style: TextStyle(
                color: Colors.blue,
                fontSize: 24,
                fontWeight: FontWeight.bold,
              ),
            ),
            Row(
              children: [
                IconButton(
                  onPressed: () async {
                    await Navigator.push(
                      context,
                      MaterialPageRoute(builder: (context) {
                        return CreatePage();
                      }),
                    );
                    showNewestPattern();
                  },
                  icon: const Icon(
                    Icons.create_outlined,
                    color: Colors.blue,
                  ),
                ),
                PatternImportButton(() {
                  showNewestPattern();
                }),
              ],
            )
          ],
        ),
        subtitle: FutureBuilder<List<Tuple2<Widget, DBImage>>>(
          future: Provider.of<Model>(context).patternDB.getImages(context),
          builder: (BuildContext context,
              AsyncSnapshot<List<Tuple2<Widget, DBImage>>> snapshot) {
            List<Widget> children;
            if (snapshot.hasData) {
              List<Tuple2<Widget, DBImage>>? tuples = snapshot.data;
              tuples ??= List.empty();
              List<Widget> widgets = List.empty(growable: true);
              for (var tuple in tuples) {
                widgets.add(
                  InkWell(
                    onTap: () async {
                      setState(() {
                        loading.value = true;
                      });
                      final poi =
                        Provider.of<Model>(context, listen: false).connectedPoi![_selectedPoiIndex];
                      if (poi.isConncted) {
                        if (!kIsWeb) {
                          // Calling connect seems to bring device to the front of a magic queue and operate faster, and properly
                          await poi.uart.device
                              .connect(timeout: const Duration(seconds: 5), autoConnect: false)
                              .timeout(const Duration(milliseconds: 5250));
                          await poi.uart.device.clearGattCache(); // Boosts speed too
                        }
                        await poi.sendPattern2(tuple.item2)
                            .timeout(const Duration(seconds: 5), onTimeout: () {
                          return false;
                        });
                      }
                      setState(() {
                        loading.value = false;
                      });
                    },
                    onLongPress: () => showDialog<void>(
                      context: context,
                      builder: (BuildContext context) => AlertDialog(
                        title: const Text("Edit/Delete Pattern"),
                        content: Text(
                          'Image Stats:\nwidth=${tuple.item2.count}\nheight=${tuple.item2.height}'),
                        actionsPadding: const EdgeInsets.all(0.0),
                        actions: <Widget>[
                          TextButton(
                            onPressed: () => Navigator.pop(context, 'Cancel'),
                            child: const Text('Cancel'),
                          ),
                          TextButton(
                            onPressed: () {
                              Navigator.pop(context, 'Flip');
                              Provider.of<Model>(context, listen: false)
                                  .patternDB
                                  .invertImage(tuple.item2.id!)
                                  .then((value) => setState(() {}));
                            },
                            child: const Text('Flip'),
                          ),
                          TextButton(
                            onPressed: () {
                              Navigator.pop(context, 'Mirror');
                              Provider.of<Model>(context, listen: false)
                                  .patternDB
                                  .reverseImage(tuple.item2.id!)
                                  .then((value) => setState(() {}));
                            },
                            child: const Text('Mirror'),
                          ),
                          TextButton(
                            onPressed: () {
                              Navigator.pop(context, 'Delete');
                              Provider.of<Model>(context, listen: false)
                                  .patternDB
                                  .deleteImage(tuple.item2.id!)
                                  .then((value) => setState(() {}));
                            },
                            child: const Text('Delete'),
                          ),
                        ],
                      ),
                    ),
                    child: Padding(
                      padding: const EdgeInsets.only(top: 8, bottom: 8),
                      child: Column(
                        mainAxisAlignment: MainAxisAlignment.center,
                        children: [
                          SingleChildScrollView(
                            scrollDirection: Axis.horizontal,
                            child: SizedBox(
                              height: 80,
                              child: tuple.item1,
                            ),
                          ),
                          const SizedBox(
                            width: 100,
                            height: 8,
                          ),
                          const Divider(
                            height: 1,
                            thickness: 1,
                            indent: 0,
                            endIndent: 0,
                          ),
                        ],
                      ),
                    ),
                  ),
                );
              }
              children = widgets;
            } else if (snapshot.hasError) {
              children = <Widget>[
                const Icon(
                  Icons.error_outline,
                  color: Colors.red,
                  size: 60,
                ),
                Padding(
                  padding: const EdgeInsets.only(top: 16),
                  child: Text('Error Loading Patterns: ${snapshot.error}'),
                ),
              ];
            } else {
              children = const <Widget>[
                SizedBox(
                  width: 60,
                  height: 60,
                  child: CircularProgressIndicator(),
                ),
                Padding(
                  padding: EdgeInsets.only(top: 16),
                  child: Text('Loading patterns...'),
                ),
              ];
            }
            return Center(
              child: Padding(
                padding: const EdgeInsets.only(bottom: 65.0),
                child: ListView(
                  controller: _scrollController,
                  children: children,
                ),
              ),
            );
          },
        ),
    );
  }

  void showNewestPattern() {
    setState(() {
      // tabIndex = 0; // This doesn't properly select the tab
    });
    // This is probably the most gross thing ive ever done, and im sorry 😭 (also the animation doesn't work)
    WidgetsBinding.instance.addPostFrameCallback((_) {
      WidgetsBinding.instance.addPostFrameCallback((_) {
        WidgetsBinding.instance.addPostFrameCallback((_) {
          _scrollController.animateTo(
            _scrollController.position.maxScrollExtent, // Scroll to the bottom
            duration: const Duration(milliseconds: 500), // Duration of the animation
            curve: Curves.easeOut, // Smooth easing curve
          );
        });
      });
    });
  }
}
