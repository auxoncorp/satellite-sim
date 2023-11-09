<template>
  <div class="viewer">
    <vc-viewer
      ref="viewer"
      :animation="animation"
      :timeline="timeline"
      :camera.sync="camera"
      :scene-mode-picker="sceneModePicker"
      :show-render-loop-errors="showRenderLoopErrors"
      :base-layer-picker="baseLayerPicker"
      :geocoder="geocoder"
      :infoBox="infoBox"
      @ready="ready"
    >
      <vc-navigation ref="navigation" :options="navOptions"></vc-navigation>
      <vc-layer-imagery></vc-layer-imagery>

      <vc-collection-primitive-polyline>
        <template v-for="(sat_tle, index) of satOrbits">
          <vc-primitive-polyline
            :show="sat_tle.show"
            :positions="sat_tle.positions"
            :key="'sat_orbit' + sat_tle.target + index"
            :material="orbitMaterial"
            :loop="true"
            :width="1"
          ></vc-primitive-polyline>
        </template>
      </vc-collection-primitive-polyline>

      <vc-collection-primitive-point>
        <template v-for="(sat_pos, index) of satPositions">
          <vc-primitive-point
            :show="sat_pos.show"
            :position="sat_pos.position"
            :key="'sat_pos' + sat_pos.target + index"
            :color="satColor"
            :pixel-size="10"
          ></vc-primitive-point>
        </template>
      </vc-collection-primitive-point>

      <vc-collection-primitive-label>
        <template v-for="(sat_pos, index) of satPositions">
          <vc-primitive-label
            :show="sat_pos.show"
            :position="sat_pos.position"
            :key="'sat_pos_label' + index"
            :text="sat_pos.target"
            :show-background="true"
            :horizontal-origin="1"
            :scale="0.6"
            :pixelOffset="{ x: 10, y: 0 }"
            :pixelOffsetScaleByDistance="{
              near: 1e1,
              nearValue: 10,
              far: 2e5,
              farValue: 1,
            }"
          >
          </vc-primitive-label>
        </template>
      </vc-collection-primitive-label>

      <vc-collection-primitive-point>
        <template v-for="(ir_event, index) of irEventPositions">
          <vc-primitive-point
            :show="ir_event.show"
            :position="ir_event.position"
            :key="'ir_event' + ir_event.id + index"
            :color="irEventColor"
            :pixel-size="5"
          ></vc-primitive-point>
        </template>
      </vc-collection-primitive-point>

      <vc-collection-primitive-label>
        <template v-for="(ir_event, index) of irEventPositions">
          <vc-primitive-label
            :show="ir_event.show"
            :position="ir_event.position"
            :key="'ir_event_label' + index"
            :text="ir_event.id.toString()"
            :show-background="true"
            :horizontal-origin="1"
            :scale="0.4"
            :pixelOffset="{ x: 5, y: 0 }"
            :pixelOffsetScaleByDistance="{
              near: 1e1,
              nearValue: 10,
              far: 2e5,
              farValue: 1,
            }"
          >
          </vc-primitive-label>
        </template>
      </vc-collection-primitive-label>
    </vc-viewer>
  </div>
</template>

<script>
import Widget from '@openc3/tool-common/src/components/widgets/Widget'
import { OpenC3Api } from '@openc3/tool-common/src/services/openc3-api.js'
import Vue from 'vue'
import VueCesium from 'vue-cesium'
import lang from 'vue-cesium/lang/en-us'
import 'vue-cesium/lib/vc-navigation.css'
import {
  ecfToEci,
  radiansToDegrees,
  sgp4,
  geodeticToEcf,
  eciToGeodetic,
  eciToEcf,
  twoline2satrec,
  gstime,
} from 'satellite.js'

Vue.use(VueCesium, {
  cesiumPath: 'https://unpkg.com/cesium@1.108/Build/Cesium/Cesium.js',
  lang: lang,
  accessToken: '',
})

export default {
  mixins: [Widget],
  data() {
    return {
      camera: {
        position: {
          lng: -77.0,
          lat: 37.0,
          height: 150000000,
        },
        heading: 360,
        pitch: -90,
        roll: 0,
      },
      animation: false,
      timeline: false,
      sceneModePicker: true,
      showRenderLoopErrors: true,
      baseLayerPicker: false,
      geocoder: false,
      infoBox: true,
      navOptions: {
        enableCompass: false,
        enableZoomControl: false,
        enableDistanceLegend: true,
        enableLocationBar: false,
        enableCompassOuterRing: false,
        enablePrintView: false,
        enableMyLocation: false,
        defaultResetView: {
          lng: -77.0,
          lat: 37.0,
          height: 150000000,
          heading: 360,
          pitch: -90,
          roll: 0,
        },
      },
      api: null,
      orbitRefreshInterval: null,
      orbitMaterial: '#FFFFFF26',
      satTleList: [],
      satPosRefreshInterval: null,
      satColor: '#FFFFFFFF',
      irEventColor: '#FF0000FF',
      satPosList: [],
      irEventRefreshInterval: null,
      irEventPosList: [],
    }
  },
  computed: {
    satOrbits() {
      return this.satTleList
    },

    satPositions() {
      return this.satPosList
    },

    irEventPositions() {
      return this.irEventPosList
    },
  },
  mounted() {
    this.$refs.viewer.createPromise.then(({ Cesium, viewer }) => {
      // Setup the low-res default provider shipped with cesium
      // TODO - the default is supposed to be this but it throws
      // a tilingScheme undefined error?
      const url =
        Cesium.buildModuleUrl('Assets/Textures/NaturalEarthII') +
        '/{z}/{x}/{reverseY}.jpg'

      for (let i = 0; i < viewer.imageryLayers.length; i++) {
        viewer.imageryLayers.remove(viewer.imageryLayers[i])
      }

      const tilingScheme = new Cesium.GeographicTilingScheme()
      const maximumLevel = 2
      const layer = viewer.imageryLayers.addImageryProvider(
        Cesium.defined(Cesium.TileMapServiceImageryProvider)
          ? new Cesium.TileMapServiceImageryProvider({
              url: url,
              tilingScheme: tilingScheme,
              maximumLevel: maximumLevel,
            })
          : Cesium.createTileMapServiceImageryProvider({
              url: url,
              tilingScheme: tilingScheme,
              maximumLevel: maximumLevel,
            })
      )
      layer.alpha = 1.0
    })
  },
  created() {
    for (let i = 0; i < this.parameters.length; i++) {
      const target = this.parameters[i]

      // Polylines for the orbits
      this.satTleList.push({
        target: target,
        positions: [],
        show: false,
      })

      // Points for the satellite positions
      this.satPosList.push({
        target: target,
        position: null,
        show: false,
        seqnum: null,
      })
    }

    const num_event_ids = 16
    for (let i = 0; i < num_event_ids; i++) {
      this.irEventPosList.push({
        id: i,
        position: null,
        show: false,
        seqnum: null,
      })
    }

    this.api = new OpenC3Api()

    this.updateSatOrbits()
    this.orbitRefreshInterval = setInterval(this.updateSatOrbits, 5 * 1000)

    this.updateSatPositions()
    this.satPosRefreshInterval = setInterval(this.updateSatPositions, 1000)

    this.updateIrEvents()
    this.irEventRefreshInterval = setInterval(this.updateIrEvents, 1000)
  },
  destroyed() {
    if (this.orbitRefreshInterval) {
      clearInterval(this.orbitRefreshInterval)
    }
    if (this.satPosRefreshInterval) {
      clearInterval(this.satPosRefreshInterval)
    }
    if (this.irEventRefreshInterval) {
      clearInterval(this.irEventRefreshInterval)
    }
  },
  methods: {
    ready(cesiumInstance) {
      this.cesiumInstance = cesiumInstance
      const { Cesium, viewer } = this.cesiumInstance
    },

    // Generate orbit polylines data once the TLE packets are available
    // NOTE: we only do this once since the source TLE isn't updated over time
    // it's only used to provide an initial orbit in the sim
    updateSatOrbits() {
      for (let i = 0; i < this.satTleList.length; i++) {
        if (this.satTleList[i].positions.length === 0) {
          const target = this.satTleList[i].target
          this.api.get_tlm_packet(target, 'TLE').then((result) => {
            const line1 = result[3][1] // LINE1
            const line2 = result[4][1] // LINE2
            if (line1 && line2) {
              var satrec = twoline2satrec(line1, line2)
              const period = (2 * Math.PI) / satrec.no
              const subdiv_minutes = 1
              var positions = []
              for (var j = 0; j < Math.floor(period); j += subdiv_minutes) {
                var positionAndVelocity = sgp4(satrec, j)
                var positionEci = positionAndVelocity.position
                positions.push({
                  x: positionEci.x * 1000.0,
                  y: positionEci.y * 1000.0,
                  z: positionEci.z * 1000.0,
                })
              }

              Vue.set(this.satTleList, i, {
                target: target,
                positions: positions,
                show: true,
              })
            }
          })
        }
      }
    },

    updateSatPositions() {
      for (let i = 0; i < this.satPosList.length; i++) {
        const target = this.satPosList[i].target
        this.api
          .get_tlm_packet(target, 'RELAY_SAT_TO_GROUND')
          .then((result) => {
            const seqnum = result[1][1] // SEQNUM
            const timeSec = result[4][1] // SAT_SEND_TIMESEC
            const timeMicroSec = result[5][1] // SAT_SEND_TIMEUS
            const lat = result[9][1] // GPS_LAT
            const lon = result[10][1] // GPS_LON
            const alt = result[11][1] // GPS_ALT
            if (seqnum && lat && lon && alt) {
              if (
                this.satPosList[i].seqnum === null ||
                this.satPosList[i].seqnum != seqnum
              ) {
                const ts = new Date(timeSec * 1000 + timeMicroSec / 1000)
                const gmst = gstime(ts)
                var positionGd = {
                  longitude: lon,
                  latitude: lat,
                  height: alt / 1000.0,
                }
                var positionEcf = geodeticToEcf(positionGd)
                var positionEci = ecfToEci(positionEcf, gmst)
                var position = {
                  x: positionEci.x * 1000.0,
                  y: positionEci.y * 1000.0,
                  z: positionEci.z * 1000.0,
                }

                Vue.set(this.satPosList, i, {
                  target: target,
                  position: position,
                  show: true,
                  seqnum: seqnum,
                })
              }
            }
          })
      }
    },

    // TODO need to add timeout mechanism, can use seqnum
    updateIrEvents() {
      for (let i = 0; i < this.irEventPosList.length; i++) {
        this.api.get_tlm_packet('CGS', 'IR_EVENT_' + i).then((result) => {
          const seqnum = result[1][1] // SEQNUM
          const x = result[5][1] // POS_X
          const y = result[6][1] // POS_Y
          const z = result[7][1] // POS_Z
          // INTENSITY [11]
          if (seqnum && x && y && z) {
            if (
              this.irEventPosList[i].seqnum === null ||
              this.irEventPosList[i].seqnum != seqnum
            ) {
              var position = {
                x: x,
                y: y,
                z: z,
              }

              Vue.set(this.irEventPosList, i, {
                id: i,
                position: position,
                show: true,
                seqnum: seqnum,
              })
            }
          }
        })
      }
    },
  },
}
</script>

<style scoped>
.viewer {
  width: 100%;
  min-width: 600px;
}
</style>
