/*
 * Generated by asn1c-0.9.29 (http://lionet.info/asn1c)
 * From ASN.1 module "NR-RRC-Definitions"
 * 	found in "/home/hpe/openairinterface5g/openair2/RRC/NR/MESSAGES/ASN.1/nr-rrc-17.3.0.asn1"
 * 	`asn1c -pdu=all -fcompound-names -gen-UPER -no-gen-BER -no-gen-JER -no-gen-OER -gen-APER -no-gen-example -findirect-choice -D ./temp`
 */

#include "ChannelAccessConfig-r16.h"

static int
memb_maxEnergyDetectionThreshold_r16_constraint_2(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -85L && value <= -52L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

static int
memb_energyDetectionThresholdOffset_r16_constraint_2(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -13L && value <= 20L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

/*
 * This type is implemented using NativeEnumerated,
 * so here we adjust the DEF accordingly.
 */
static int
memb_ul_toDL_COT_SharingED_Threshold_r16_constraint_1(const asn_TYPE_descriptor_t *td, const void *sptr,
			asn_app_constraint_failed_f *ctfailcb, void *app_key) {
	long value;
	
	if(!sptr) {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: value not given (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
	
	value = *(const long *)sptr;
	
	if((value >= -85L && value <= -52L)) {
		/* Constraint check succeeded */
		return 0;
	} else {
		ASN__CTFAIL(app_key, td, sptr,
			"%s: constraint failed (%s:%d)",
			td->name, __FILE__, __LINE__);
		return -1;
	}
}

#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_maxEnergyDetectionThreshold_r16_constr_3 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 6,  6, -85, -52 }	/* (-85..-52) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_energyDetectionThresholdOffset_r16_constr_4 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 6,  6, -13,  20 }	/* (-13..20) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_energyDetectionConfig_r16_constr_2 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 1,  1,  0,  1 }	/* (0..1) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_type_absenceOfAnyOtherTechnology_r16_constr_6 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 0,  0,  0,  0 }	/* (0..0) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
static asn_per_constraints_t asn_PER_memb_ul_toDL_COT_SharingED_Threshold_r16_constr_5 CC_NOTUSED = {
	{ APC_CONSTRAINED,	 6,  6, -85, -52 }	/* (-85..-52) */,
	{ APC_UNCONSTRAINED,	-1, -1,  0,  0 },
	0, 0	/* No PER value map */
};
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
static asn_TYPE_member_t asn_MBR_energyDetectionConfig_r16_2[] = {
	{ ATF_NOFLAGS, 0, offsetof(struct ChannelAccessConfig_r16__energyDetectionConfig_r16, choice.maxEnergyDetectionThreshold_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_maxEnergyDetectionThreshold_r16_constr_3,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_maxEnergyDetectionThreshold_r16_constraint_2
		},
		0, 0, /* No default value */
		"maxEnergyDetectionThreshold-r16"
		},
	{ ATF_NOFLAGS, 0, offsetof(struct ChannelAccessConfig_r16__energyDetectionConfig_r16, choice.energyDetectionThresholdOffset_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_energyDetectionThresholdOffset_r16_constr_4,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_energyDetectionThresholdOffset_r16_constraint_2
		},
		0, 0, /* No default value */
		"energyDetectionThresholdOffset-r16"
		},
};
static const asn_TYPE_tag2member_t asn_MAP_energyDetectionConfig_r16_tag2el_2[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* maxEnergyDetectionThreshold-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 } /* energyDetectionThresholdOffset-r16 */
};
static asn_CHOICE_specifics_t asn_SPC_energyDetectionConfig_r16_specs_2 = {
	sizeof(struct ChannelAccessConfig_r16__energyDetectionConfig_r16),
	offsetof(struct ChannelAccessConfig_r16__energyDetectionConfig_r16, _asn_ctx),
	offsetof(struct ChannelAccessConfig_r16__energyDetectionConfig_r16, present),
	sizeof(((struct ChannelAccessConfig_r16__energyDetectionConfig_r16 *)0)->present),
	asn_MAP_energyDetectionConfig_r16_tag2el_2,
	2,	/* Count of tags in the map */
	0, 0,
	-1	/* Extensions start */
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_energyDetectionConfig_r16_2 = {
	"energyDetectionConfig-r16",
	"energyDetectionConfig-r16",
	&asn_OP_CHOICE,
	0,	/* No effective tags (pointer) */
	0,	/* No effective tags (count) */
	0,	/* No tags (pointer) */
	0,	/* No tags (count) */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_energyDetectionConfig_r16_constr_2,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		CHOICE_constraint
	},
	asn_MBR_energyDetectionConfig_r16_2,
	2,	/* Elements count */
	&asn_SPC_energyDetectionConfig_r16_specs_2	/* Additional specs */
};

static const asn_INTEGER_enum_map_t asn_MAP_absenceOfAnyOtherTechnology_r16_value2enum_6[] = {
	{ 0,	4,	"true" }
};
static const unsigned int asn_MAP_absenceOfAnyOtherTechnology_r16_enum2value_6[] = {
	0	/* true(0) */
};
static const asn_INTEGER_specifics_t asn_SPC_absenceOfAnyOtherTechnology_r16_specs_6 = {
	asn_MAP_absenceOfAnyOtherTechnology_r16_value2enum_6,	/* "tag" => N; sorted by tag */
	asn_MAP_absenceOfAnyOtherTechnology_r16_enum2value_6,	/* N => "tag"; sorted by N */
	1,	/* Number of elements in the maps */
	0,	/* Enumeration is not extensible */
	1,	/* Strict enumeration */
	0,	/* Native long size */
	0
};
static const ber_tlv_tag_t asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6[] = {
	(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
	(ASN_TAG_CLASS_UNIVERSAL | (10 << 2))
};
static /* Use -fall-defs-global to expose */
asn_TYPE_descriptor_t asn_DEF_absenceOfAnyOtherTechnology_r16_6 = {
	"absenceOfAnyOtherTechnology-r16",
	"absenceOfAnyOtherTechnology-r16",
	&asn_OP_NativeEnumerated,
	asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6,
	sizeof(asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6)
		/sizeof(asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6[0]) - 1, /* 1 */
	asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6,	/* Same as above */
	sizeof(asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6)
		/sizeof(asn_DEF_absenceOfAnyOtherTechnology_r16_tags_6[0]), /* 2 */
	{
#if !defined(ASN_DISABLE_OER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
		&asn_PER_type_absenceOfAnyOtherTechnology_r16_constr_6,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
		0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
		NativeEnumerated_constraint
	},
	0, 0,	/* Defined elsewhere */
	&asn_SPC_absenceOfAnyOtherTechnology_r16_specs_6	/* Additional specs */
};

asn_TYPE_member_t asn_MBR_ChannelAccessConfig_r16_1[] = {
	{ ATF_POINTER, 3, offsetof(struct ChannelAccessConfig_r16, energyDetectionConfig_r16),
		(ASN_TAG_CLASS_CONTEXT | (0 << 2)),
		+1,	/* EXPLICIT tag at current level */
		&asn_DEF_energyDetectionConfig_r16_2,
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
		"energyDetectionConfig-r16"
		},
	{ ATF_POINTER, 2, offsetof(struct ChannelAccessConfig_r16, ul_toDL_COT_SharingED_Threshold_r16),
		(ASN_TAG_CLASS_CONTEXT | (1 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_NativeInteger,
		0,
		{
#if !defined(ASN_DISABLE_OER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_OER_SUPPORT) */
#if !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT)
			&asn_PER_memb_ul_toDL_COT_SharingED_Threshold_r16_constr_5,
#endif  /* !defined(ASN_DISABLE_UPER_SUPPORT) || !defined(ASN_DISABLE_APER_SUPPORT) */
#if !defined(ASN_DISABLE_JER_SUPPORT)
			0,
#endif  /* !defined(ASN_DISABLE_JER_SUPPORT) */
			memb_ul_toDL_COT_SharingED_Threshold_r16_constraint_1
		},
		0, 0, /* No default value */
		"ul-toDL-COT-SharingED-Threshold-r16"
		},
	{ ATF_POINTER, 1, offsetof(struct ChannelAccessConfig_r16, absenceOfAnyOtherTechnology_r16),
		(ASN_TAG_CLASS_CONTEXT | (2 << 2)),
		-1,	/* IMPLICIT tag at current level */
		&asn_DEF_absenceOfAnyOtherTechnology_r16_6,
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
		"absenceOfAnyOtherTechnology-r16"
		},
};
static const int asn_MAP_ChannelAccessConfig_r16_oms_1[] = { 0, 1, 2 };
static const ber_tlv_tag_t asn_DEF_ChannelAccessConfig_r16_tags_1[] = {
	(ASN_TAG_CLASS_UNIVERSAL | (16 << 2))
};
static const asn_TYPE_tag2member_t asn_MAP_ChannelAccessConfig_r16_tag2el_1[] = {
    { (ASN_TAG_CLASS_CONTEXT | (0 << 2)), 0, 0, 0 }, /* energyDetectionConfig-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (1 << 2)), 1, 0, 0 }, /* ul-toDL-COT-SharingED-Threshold-r16 */
    { (ASN_TAG_CLASS_CONTEXT | (2 << 2)), 2, 0, 0 } /* absenceOfAnyOtherTechnology-r16 */
};
asn_SEQUENCE_specifics_t asn_SPC_ChannelAccessConfig_r16_specs_1 = {
	sizeof(struct ChannelAccessConfig_r16),
	offsetof(struct ChannelAccessConfig_r16, _asn_ctx),
	asn_MAP_ChannelAccessConfig_r16_tag2el_1,
	3,	/* Count of tags in the map */
	asn_MAP_ChannelAccessConfig_r16_oms_1,	/* Optional members */
	3, 0,	/* Root/Additions */
	-1,	/* First extension addition */
};
asn_TYPE_descriptor_t asn_DEF_ChannelAccessConfig_r16 = {
	"ChannelAccessConfig-r16",
	"ChannelAccessConfig-r16",
	&asn_OP_SEQUENCE,
	asn_DEF_ChannelAccessConfig_r16_tags_1,
	sizeof(asn_DEF_ChannelAccessConfig_r16_tags_1)
		/sizeof(asn_DEF_ChannelAccessConfig_r16_tags_1[0]), /* 1 */
	asn_DEF_ChannelAccessConfig_r16_tags_1,	/* Same as above */
	sizeof(asn_DEF_ChannelAccessConfig_r16_tags_1)
		/sizeof(asn_DEF_ChannelAccessConfig_r16_tags_1[0]), /* 1 */
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
	asn_MBR_ChannelAccessConfig_r16_1,
	3,	/* Elements count */
	&asn_SPC_ChannelAccessConfig_r16_specs_1	/* Additional specs */
};

