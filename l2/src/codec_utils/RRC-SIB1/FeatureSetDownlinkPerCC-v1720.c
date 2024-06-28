/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "FeatureSetDownlinkPerCC-v1720.h"

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_maxModulationOrderForMulticastDataRateCalculation_r17_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 2,  2,  0,  2 }	/* (0..2) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_fdm_BroadcastUnicast_r17_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_fdm_MulticastUnicast_r17_constr_8 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_maxModulationOrderForMulticastDataRateCalculation_r17_value2enum_2[] = {
	{ 0,	5,	"qam64" },
	{ 1,	6,	"qam256" },
	{ 2,	7,	"qam1024" }
};
static const unsigned int asn_MAP_maxModulationOrderForMulticastDataRateCalculation_r17_enum2value_2[] = {
	2,	/* qam1024(2) */
	1,	/* qam256(1) */
	0	/* qam64(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_maxModulationOrderForMulticastDataRateCalculation_r17_specs_2 = {
	asn_MAP_maxModulationOrderForMulticastDataRateCalculation_r17_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_maxModulationOrderForMulticastDataRateCalculation_r17_enum2value_2,	/* N => "tag"; sorted by N */
	3,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_2 = {
	"maxModulationOrderForMulticastDataRateCalculation-r17",
	"maxModulationOrderForMulticastDataRateCalculation-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2,
	sizeof(asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2)
		/sizeof(asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2)
		/sizeof(asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_maxModulationOrderForMulticastDataRateCalculation_r17_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_maxModulationOrderForMulticastDataRateCalculation_r17_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_fdm_BroadcastUnicast_r17_value2enum_6[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_fdm_BroadcastUnicast_r17_enum2value_6[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_fdm_BroadcastUnicast_r17_specs_6 = {
	asn_MAP_fdm_BroadcastUnicast_r17_value2enum_6,	/* "tag" => N; sorted by tag */
	asn_MAP_fdm_BroadcastUnicast_r17_enum2value_6,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_fdm_BroadcastUnicast_r17_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_fdm_BroadcastUnicast_r17_6 = {
	"fdm-BroadcastUnicast-r17",
	"fdm-BroadcastUnicast-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_fdm_BroadcastUnicast_r17_tags_6,
	sizeof(asn_DEF_fdm_BroadcastUnicast_r17_tags_6)
		/sizeof(asn_DEF_fdm_BroadcastUnicast_r17_tags_6[0]) - 1, /* 1 */
	asn_DEF_fdm_BroadcastUnicast_r17_tags_6,	/* Same as above */
	sizeof(asn_DEF_fdm_BroadcastUnicast_r17_tags_6)
		/sizeof(asn_DEF_fdm_BroadcastUnicast_r17_tags_6[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_fdm_BroadcastUnicast_r17_constr_6,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_fdm_BroadcastUnicast_r17_specs_6	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_fdm_MulticastUnicast_r17_value2enum_8[] = {
	{ 0,	9,	"supported" }
};
static const unsigned int asn_MAP_fdm_MulticastUnicast_r17_enum2value_8[] = {
	0	/* supported(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_fdm_MulticastUnicast_r17_specs_8 = {
	asn_MAP_fdm_MulticastUnicast_r17_value2enum_8,	/* "tag" => N; sorted by tag */
	asn_MAP_fdm_MulticastUnicast_r17_enum2value_8,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_fdm_MulticastUnicast_r17_tags_8[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_fdm_MulticastUnicast_r17_8 = {
	"fdm-MulticastUnicast-r17",
	"fdm-MulticastUnicast-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_fdm_MulticastUnicast_r17_tags_8,
	sizeof(asn_DEF_fdm_MulticastUnicast_r17_tags_8)
		/sizeof(asn_DEF_fdm_MulticastUnicast_r17_tags_8[0]) - 1, /* 1 */
	asn_DEF_fdm_MulticastUnicast_r17_tags_8,	/* Same as above */
	sizeof(asn_DEF_fdm_MulticastUnicast_r17_tags_8)
		/sizeof(asn_DEF_fdm_MulticastUnicast_r17_tags_8[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_fdm_MulticastUnicast_r17_constr_8,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_fdm_MulticastUnicast_r17_specs_8	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_FeatureSetDownlinkPerCC_v1720_1[] = {
	{ ATF_POINTER, 3, offsetof(struct FeatureSetDownlinkPerCC_v1720, maxModulationOrderForMulticastDataRateCalculation_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_maxModulationOrderForMulticastDataRateCalculation_r17_2,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"maxModulationOrderForMulticastDataRateCalculation-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct FeatureSetDownlinkPerCC_v1720, fdm_BroadcastUnicast_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_fdm_BroadcastUnicast_r17_6,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"fdm-BroadcastUnicast-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct FeatureSetDownlinkPerCC_v1720, fdm_MulticastUnicast_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_fdm_MulticastUnicast_r17_8,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			0
		},
		0, 0, /* No default value */
		"fdm-MulticastUnicast-r17"
		},
};
static const int asn_MAP_FeatureSetDownlinkPerCC_v1720_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_FeatureSetDownlinkPerCC_v1720_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* maxModulationOrderForMulticastDataRateCalculation-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* fdm-BroadcastUnicast-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* fdm-MulticastUnicast-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_FeatureSetDownlinkPerCC_v1720_specs_1 = {
	sizeof(struct FeatureSetDownlinkPerCC_v1720),
	offsetof(struct FeatureSetDownlinkPerCC_v1720, _asn_ctx),
	asn_MAP_FeatureSetDownlinkPerCC_v1720_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_FeatureSetDownlinkPerCC_v1720_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_FeatureSetDownlinkPerCC_v1720 = {
	"FeatureSetDownlinkPerCC-v1720",
	"FeatureSetDownlinkPerCC-v1720",
	&asn_OP_SEQUENCE,
	asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1,
	sizeof(asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1)
		/sizeof(asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1[0]), /* 1 */
	asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1,	/* Same as above */
	sizeof(asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1)
		/sizeof(asn_DEF_FeatureSetDownlinkPerCC_v1720_tags_1[0]), /* 1 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		SEQUENCE_constraint
	},
	asn_MBR_FeatureSetDownlinkPerCC_v1720_1,
	3,	/* Elements count */
	&asn_SPC_FeatureSetDownlinkPerCC_v1720_specs_1	/* Additional specs */
};

