Shader "Custom/PointCloudFused" {
	Properties {
		[NoScaleOffset]_UVMapA ("UV Map A", 2D) = "black" {}
		[NoScaleOffset]_MainTexA ("Color A", 2D) = "white" {}
		[NoScaleOffset]_UVMapB ("UV Map B", 2D) = "black" {}
		[NoScaleOffset]_MainTexB ("Color B", 2D) = "white" {}
		_PointSize("Point Size", Float) = 10.0
		_Color ("Tint", Color) = (1, 1, 1, 1)
		[Toggle(USE_DISTANCE)]_UseDistance ("Scale by distance?", float) = 1
	}

	SubShader
	{
		Tags { "RenderType"="Opaque" }
		Cull Off
		ZWrite On
		ZTest LEqual

		Pass
		{
			CGPROGRAM
			#pragma vertex vert
			#pragma geometry geom
			#pragma fragment frag
			#pragma shader_feature USE_DISTANCE
			#include "UnityCG.cginc"

			struct appdata
			{
				float4 vertex : POSITION;
				float2 uv : TEXCOORD0;
				float2 uv2 : TEXCOORD1;
			};

			struct v2g
			{
				float4 vertex : TEXCOORD0;
				float2 uv : TEXCOORD1;
				float cloudId : TEXCOORD2;
			};

			struct g2f
			{
				float4 vertex : SV_POSITION;
				float2 uv : TEXCOORD0;
				float cloudId : TEXCOORD1;
				float2 quadUV : TEXCOORD2;
			};

			float _PointSize;
			fixed4 _Color;

			sampler2D _UVMapA;
			sampler2D _MainTexA;
			float4 _MainTexA_TexelSize;

			sampler2D _UVMapB;
			sampler2D _MainTexB;
			float4 _MainTexB_TexelSize;

			v2g vert(appdata v)
			{
				v2g o;
				o.vertex = v.vertex;
				o.vertex.y = -o.vertex.y;
				o.uv = v.uv;
				o.cloudId = v.uv2.x;
				return o;
			}

			[maxvertexcount(4)]
			void geom(point v2g i[1], inout TriangleStream<g2f> triStream)
			{
				float4 v = i[0].vertex;
				if (abs(v.x) + abs(v.y) + abs(v.z) < 0.0001)
					return;

				g2f o;
				float2 uv = i[0].uv;
				o.cloudId = i[0].cloudId;

				float4 clip = UnityObjectToClipPos(v);
				float2 p = _PointSize * 0.001;
				p.y *= _ScreenParams.x / _ScreenParams.y;

				#ifndef USE_DISTANCE
				p *= clip.w;
				#endif

				o.uv = uv;

				o.vertex = clip + float4(-p.x, p.y, 0, 0);
				o.quadUV = float2(0, 1);
				triStream.Append(o);

				o.vertex = clip + float4(-p.x, -p.y, 0, 0);
				o.quadUV = float2(0, 0);
				triStream.Append(o);

				o.vertex = clip + float4(p.x, p.y, 0, 0);
				o.quadUV = float2(1, 1);
				triStream.Append(o);

				o.vertex = clip + float4(p.x, -p.y, 0, 0);
				o.quadUV = float2(1, 0);
				triStream.Append(o);
			}

			fixed4 frag(g2f i) : SV_Target
			{
				// Circular disc clipping: discard corners outside radius
				float2 centered = i.quadUV - 0.5;
				float distSq = dot(centered, centered);
				if (distSq > 0.25)
					discard;

				float2 colorUV;
				if (i.cloudId < 0.5)
				{
					colorUV = tex2D(_UVMapA, i.uv).rg;
					if (any(colorUV <= 0 || colorUV >= 1))
						discard;
					colorUV += 0.5 * _MainTexA_TexelSize.xy;
					return tex2D(_MainTexA, colorUV) * _Color;
				}
				else
				{
					colorUV = tex2D(_UVMapB, i.uv).rg;
					if (any(colorUV <= 0 || colorUV >= 1))
						discard;
					colorUV += 0.5 * _MainTexB_TexelSize.xy;
					return tex2D(_MainTexB, colorUV) * _Color;
				}
			}
			ENDCG
		}
	}
}
