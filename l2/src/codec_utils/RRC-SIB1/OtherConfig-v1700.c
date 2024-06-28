/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "OtherConfig-v1700.h"

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
/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_ul_GapFR2_PreferenceConfig_r17_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_maxBW_PreferenceConfigFR2_2_r17_constr_7 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_maxMIMO_LayerPreferenceConfigFR2_2_r17_constr_9 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_minSchedulingOffsetPreferenceConfigExt_r17_constr_11 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static const asn_INTEGER_enum_map_t asn_MAP_ul_GapFR2_PreferenceConfig_r17_value2enum_2[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_ul_GapFR2_PreferenceConfig_r17_enum2value_2[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_ul_GapFR2_PreferenceConfig_r17_specs_2 = {
	asn_MAP_ul_GapFR2_PreferenceConfig_r17_value2enum_2,	/* "tag" => N; sorted by tag */
	asn_MAP_ul_GapFR2_PreferenceConfig_r17_enum2value_2,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2[] = {
	(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_ul_GapFR2_PreferenceConfig_r17_2 = {
	"ul-GapFR2-PreferenceConfig-r17",
	"ul-GapFR2-PreferenceConfig-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2,
	sizeof(asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2)
		/sizeof(asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2[0]) - 1, /* 1 */
	asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2,	/* Same as above */
	sizeof(asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2)
		/sizeof(asn_DEF_ul_GapFR2_PreferenceConfig_r17_tags_2[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_ul_GapFR2_PreferenceConfig_r17_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_ul_GapFR2_PreferenceConfig_r17_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_maxBW_PreferenceConfigFR2_2_r17_value2enum_7[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_maxBW_PreferenceConfigFR2_2_r17_enum2value_7[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_maxBW_PreferenceConfigFR2_2_r17_specs_7 = {
	asn_MAP_maxBW_PreferenceConfigFR2_2_r17_value2enum_7,	/* "tag" => N; sorted by tag */
	asn_MAP_maxBW_PreferenceConfigFR2_2_r17_enum2value_7,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7[] = {
	(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxBW_PreferenceConfigFR2_2_r17_7 = {
	"maxBW-PreferenceConfigFR2-2-r17",
	"maxBW-PreferenceConfigFR2-2-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7,
	sizeof(asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7)
		/sizeof(asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7[0]) - 1, /* 1 */
	asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7,	/* Same as above */
	sizeof(asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7)
		/sizeof(asn_DEF_maxBW_PreferenceConfigFR2_2_r17_tags_7[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_maxBW_PreferenceConfigFR2_2_r17_constr_7,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_maxBW_PreferenceConfigFR2_2_r17_specs_7	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_maxMIMO_LayerPreferenceConfigFR2_2_r17_value2enum_9[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_maxMIMO_LayerPreferenceConfigFR2_2_r17_enum2value_9[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_maxMIMO_LayerPreferenceConfigFR2_2_r17_specs_9 = {
	asn_MAP_maxMIMO_LayerPreferenceConfigFR2_2_r17_value2enum_9,	/* "tag" => N; sorted by tag */
	asn_MAP_maxMIMO_LayerPreferenceConfigFR2_2_r17_enum2value_9,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9[] = {
	(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_9 = {
	"maxMIMO-LayerPreferenceConfigFR2-2-r17",
	"maxMIMO-LayerPreferenceConfigFR2-2-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9,
	sizeof(asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9)
		/sizeof(asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9[0]) - 1, /* 1 */
	asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9,	/* Same as above */
	sizeof(asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9)
		/sizeof(asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_tags_9[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_maxMIMO_LayerPreferenceConfigFR2_2_r17_constr_9,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_maxMIMO_LayerPreferenceConfigFR2_2_r17_specs_9	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_minSchedulingOffsetPreferenceConfigExt_r17_value2enum_11[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_minSchedulingOffsetPreferenceConfigExt_r17_enum2value_11[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_minSchedulingOffsetPreferenceConfigExt_r17_specs_11 = {
	asn_MAP_minSchedulingOffsetPreferenceConfigExt_r17_value2enum_11,	/* "tag" => N; sorted by tag */
	asn_MAP_minSchedulingOffsetPreferenceConfigExt_r17_enum2value_11,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11[] = {
	(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_11 = {
	"minSchedulingOffsetPreferenceConfigExt-r17",
	"minSchedulingOffsetPreferenceConfigExt-r17",
	&asn_OP_NativeEnumerated,
	asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11,
	sizeof(asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11)
		/sizeof(asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11[0]) - 1, /* 1 */
	asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11,	/* Same as above */
	sizeof(asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11)
		/sizeof(asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_tags_11[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_minSchedulingOffsetPreferenceConfigExt_r17_constr_11,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_minSchedulingOffsetPreferenceConfigExt_r17_specs_11	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_OtherConfig_v1700_1[] = {
	{ ATF_POINTER, 12, offsetof(struct OtherConfig_v1700, ul_GapFR2_PreferenceConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_ul_GapFR2_PreferenceConfig_r17_2,
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
		"ul-GapFR2-PreferenceConfig-r17"
		},
	{ ATF_POINTER, 11, offsetof(struct OtherConfig_v1700, musim_GapAssistanceConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_MUSIM_GapAssistanceConfig_r17,
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
		"musim-GapAssistanceConfig-r17"
		},
	{ ATF_POINTER, 10, offsetof(struct OtherConfig_v1700, musim_LeaveAssistanceConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_MUSIM_LeaveAssistanceConfig_r17,
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
		"musim-LeaveAssistanceConfig-r17"
		},
	{ ATF_POINTER, 9, offsetof(struct OtherConfig_v1700, successHO_Config_r17),
		(ASN_TAG_CLASS_CONTEXT | (3 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_SuccessHO_Config_r17,
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
		"successHO-Config-r17"
		},
	{ ATF_POINTER, 8, offsetof(struct OtherConfig_v1700, maxBW_PreferenceConfigFR2_2_r17),
		(ASN_TAG_CLASS_CONTEXT | (4 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_maxBW_PreferenceConfigFR2_2_r17_7,
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
		"maxBW-PreferenceConfigFR2-2-r17"
		},
	{ ATF_POINTER, 7, offsetof(struct OtherConfig_v1700, maxMIMO_LayerPreferenceConfigFR2_2_r17),
		(ASN_TAG_CLASS_CONTEXT | (5 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_maxMIMO_LayerPreferenceConfigFR2_2_r17_9,
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
		"maxMIMO-LayerPreferenceConfigFR2-2-r17"
		},
	{ ATF_POINTER, 6, offsetof(struct OtherConfig_v1700, minSchedulingOffsetPreferenceConfigExt_r17),
		(ASN_TAG_CLASS_CONTEXT | (6 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_minSchedulingOffsetPreferenceConfigExt_r17_11,
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
		"minSchedulingOffsetPreferenceConfigExt-r17"
		},
	{ ATF_POINTER, 5, offsetof(struct OtherConfig_v1700, rlm_RelaxationReportingConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (7 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_RLM_RelaxationReportingConfig_r17,
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
		"rlm-RelaxationReportingConfig-r17"
		},
	{ ATF_POINTER, 4, offsetof(struct OtherConfig_v1700, bfd_RelaxationReportingConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (8 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_BFD_RelaxationReportingConfig_r17,
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
		"bfd-RelaxationReportingConfig-r17"
		},
	{ ATF_POINTER, 3, offsetof(struct OtherConfig_v1700, scg_DeactivationPreferenceConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (9 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_SCG_DeactivationPreferenceConfig_r17,
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
		"scg-DeactivationPreferenceConfig-r17"
		},
	{ ATF_POINTER, 2, offsetof(struct OtherConfig_v1700, rrm_MeasRelaxationReportingConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (10 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_RRM_MeasRelaxationReportingConfig_r17,
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
		"rrm-MeasRelaxationReportingConfig-r17"
		},
	{ ATF_POINTER, 1, offsetof(struct OtherConfig_v1700, propDelayDiffReportConfig_r17),
		(ASN_TAG_CLASS_CONTEXT | (11 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_SetupRelease_PropDelayDiffReportConfig_r17,
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
		"propDelayDiffReportConfig-r17"
		},
};
static const int asn_MAP_OtherConfig_v1700_oms_1[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11 };
static const ber_tlv_tag_t asn_DEF_OtherConfig_v1700_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_OtherConfig_v1700_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* ul-GapFR2-PreferenceConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* musim-GapAssistanceConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 }, /* musim-LeaveAssistanceConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (3 << 2)), 3, 0, 0 }, /* successHO-Config-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (4 << 2)), 4, 0, 0 }, /* maxBW-PreferenceConfigFR2-2-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (5 << 2)), 5, 0, 0 }, /* maxMIMO-LayerPreferenceConfigFR2-2-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (6 << 2)), 6, 0, 0 }, /* minSchedulingOffsetPreferenceConfigExt-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (7 << 2)), 7, 0, 0 }, /* rlm-RelaxationReportingConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (8 << 2)), 8, 0, 0 }, /* bfd-RelaxationReportingConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (9 << 2)), 9, 0, 0 }, /* scg-DeactivationPreferenceConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (10 << 2)), 10, 0, 0 }, /* rrm-MeasRelaxationReportingConfig-r17 */
    { (ASN_TAG_CLASS_CONTEXT | (11 << 2)), 11, 0, 0 } /* propDelayDiffReportConfig-r17 */
};
asn_SEQUENCE_specifics_t asn_SPC_OtherConfig_v1700_specs_1 = {
	sizeof(struct OtherConfig_v1700),
	offsetof(struct OtherConfig_v1700, _asn_ctx),
	asn_MAP_OtherConfig_v1700_tag2el_1,
	12,	/* Count of tags in the map */
	asn_MAP_OtherConfig_v1700_oms_1,	/* Optional members */
	12, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_OtherConfig_v1700 = {
	"OtherConfig-v1700",
	"OtherConfig-v1700",
	&asn_OP_SEQUENCE,
	asn_DEF_OtherConfig_v1700_tags_1,
	sizeof(asn_DEF_OtherConfig_v1700_tags_1)
		/sizeof(asn_DEF_OtherConfig_v1700_tags_1[0]), /* 1 */
	asn_DEF_OtherConfig_v1700_tags_1,	/* Same as above */
	sizeof(asn_DEF_OtherConfig_v1700_tags_1)
		/sizeof(asn_DEF_OtherConfig_v1700_tags_1[0]), /* 1 */
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
	asn_MBR_OtherConfig_v1700_1,
	12,	/* Elements count */
	&asn_SPC_OtherConfig_v1700_specs_1	/* Additional specs */
};

